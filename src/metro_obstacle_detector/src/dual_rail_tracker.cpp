#include "metro_obstacle_detector/dual_rail_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace metro_obstacle_detector
{

DualRailTracker::DualRailTracker(const DualRailConfig & config)
: config_(config),
  last_valid_crown_z_(-config.nominal_tor_height)
{
}

void DualRailTracker::set_config(const DualRailConfig & config)
{
  config_ = config;
  last_valid_crown_z_ = -config.nominal_tor_height;
}

void DualRailTracker::reset()
{
  last_valid_crown_z_ = -config_.nominal_tor_height;
  dynamic_grade_slope_ = 0.0f;
}

DualRailMeasurement DualRailTracker::estimate_slice(
  const std::vector<Point3D> & slice_points,
  float x_prior,
  float y_center,
  float z_prior,
  float yaw_prior)
{
  DualRailMeasurement meas;
  meas.crown_z = z_prior;
  meas.left_rail_z = z_prior;
  meas.right_rail_z = z_prior;
  meas.track_center_x = x_prior;

  const float nominal_g = config_.gauge;               // 1.520m
  const float half_g = 0.5f * nominal_g;               // 0.760m
  const float g_tol = config_.gauge_tolerance;         // 0.080m (+-80mm)
  const float half_window = config_.rail_search_half_width; // 0.14m

  const float cos_y = std::cos(yaw_prior);
  const float sin_y = std::sin(yaw_prior);

  // Collect candidate points in track-aligned coordinate frame (u, v_tangent, z)
  struct RailCandPt {
    float x, y, z;
    float u;
  };

  std::vector<RailCandPt> left_pts;
  std::vector<RailCandPt> right_pts;
  std::vector<float> bed_zs;

  left_pts.reserve(128);
  right_pts.reserve(128);
  bed_zs.reserve(256);

  const float z_low = z_prior - 0.45f;
  const float z_high = z_prior + 0.30f;

  for (const auto & p : slice_points) {
    const float dx = p.x - x_prior;
    const float dy = p.y - y_center;
    const float u = dx * cos_y + dy * sin_y; // Lateral displacement from prior track center
    const float v_tangent = -dx * sin_y + dy * cos_y; // Tangential along track
    const float pz = p.z;

    if (pz < z_low || pz > z_high) continue;
    if (std::abs(v_tangent) > 1.25f) continue; // Slice longitudinal boundary

    // 1. Central track bed / drainage trough
    if (std::abs(u) <= 0.40f) {
      bed_zs.push_back(pz);
    }

    // 2. Left rail search window: u around -half_g (-0.76m +- 0.14m)
    if (std::abs(u - (-half_g)) <= half_window) {
      left_pts.push_back(RailCandPt{p.x, p.y, pz, u});
    }
    // 3. Right rail search window: u around +half_g (+0.76m +- 0.14m)
    else if (std::abs(u - half_g) <= half_window) {
      right_pts.push_back(RailCandPt{p.x, p.y, pz, u});
    }
  }

  // Track bed estimation
  if (!bed_zs.empty()) {
    std::sort(bed_zs.begin(), bed_zs.end());
    meas.bed_z = bed_zs[bed_zs.size() / 2];
  } else {
    meas.bed_z = z_prior - config_.bed_to_rail_height;
  }

  // Binning helper for rail head peak extraction
  struct RailBin {
    float u_mean{0.0f};
    float x_mean{0.0f};
    float z_crown{0.0f};
    size_t count{0};
  };

  auto extract_bins = [](const std::vector<RailCandPt> & pts, float min_u, float max_u, float bin_w) {
    std::vector<RailBin> bins;
    const int num_bins = std::max(1, static_cast<int>(std::ceil((max_u - min_u) / bin_w)));
    std::vector<std::vector<const RailCandPt *>> binned(num_bins);

    for (const auto & p : pts) {
      int idx = static_cast<int>((p.u - min_u) / bin_w);
      if (idx >= 0 && idx < num_bins) {
        binned[idx].push_back(&p);
      }
    }

    for (int i = 0; i < num_bins; ++i) {
      if (binned[i].size() >= 2) {
        std::vector<float> zs;
        zs.reserve(binned[i].size());
        double sum_u = 0.0, sum_x = 0.0;
        for (const auto * p : binned[i]) {
          zs.push_back(p->z);
          sum_u += p->u;
          sum_x += p->x;
        }
        std::sort(zs.begin(), zs.end());
        const size_t q_idx = static_cast<size_t>(zs.size() * 0.90f);
        const double inv = 1.0 / static_cast<double>(binned[i].size());

        bins.push_back(RailBin{
          static_cast<float>(sum_u * inv),
          static_cast<float>(sum_x * inv),
          zs[q_idx],
          zs.size()
        });
      }
    }
    return bins;
  };

  const float bin_width = 0.025f; // 25 mm lateral resolution
  const auto left_bins = extract_bins(left_pts, -half_g - half_window, -half_g + half_window, bin_width);
  const auto right_bins = extract_bins(right_pts, half_g - half_window, half_g + half_window, bin_width);

  // Dual-rail pairwise correlation matching
  float best_score = -1.0f;
  int best_l = -1;
  int best_r = -1;

  for (size_t i = 0; i < left_bins.size(); ++i) {
    for (size_t j = 0; j < right_bins.size(); ++j) {
      const float measured_gauge = right_bins[j].u_mean - left_bins[i].u_mean;
      const float gauge_err = std::abs(measured_gauge - nominal_g);

      // Verify gauge tolerance: G = 1520 mm (+-80 mm)
      if (gauge_err > g_tol) continue;

      const float delta_z = std::abs(right_bins[j].z_crown - left_bins[i].z_crown);
      // Railway superelevation / cant limit (max cant ~ 0.12 m)
      if (delta_z > 0.12f) continue;

      // Both rails should be elevated above the central track bed
      if (left_bins[i].z_crown - meas.bed_z < 0.06f || right_bins[j].z_crown - meas.bed_z < 0.06f) {
        continue;
      }

      // Correlation score penalizing deviation from standard gauge and large roll asymmetry
      const float sigma_g = 0.035f;
      const float sigma_z = 0.040f;
      const float score = std::sqrt(static_cast<float>(left_bins[i].count * right_bins[j].count)) *
        std::exp(-0.5f * (gauge_err * gauge_err) / (sigma_g * sigma_g)) *
        std::exp(-0.5f * (delta_z * delta_z) / (sigma_z * sigma_z));

      if (score > best_score) {
        best_score = score;
        best_l = static_cast<int>(i);
        best_r = static_cast<int>(j);
      }
    }
  }

  if (best_l >= 0 && best_r >= 0 && best_score > 1.5f) {
    // Both rails successfully identified through dual correlation!
    const auto & lb = left_bins[best_l];
    const auto & rb = right_bins[best_r];

    meas.left_rail_x = lb.x_mean;
    meas.left_rail_z = lb.z_crown;
    meas.left_detected = true;

    meas.right_rail_x = rb.x_mean;
    meas.right_rail_z = rb.z_crown;
    meas.right_detected = true;

    // Rail crown (УГР) is defined as top of the highest rail in track cross-section
    meas.crown_z = std::max(meas.left_rail_z, meas.right_rail_z);
    meas.track_center_x = 0.5f * (meas.left_rail_x + meas.right_rail_x);

    const float g_actual = rb.u_mean - lb.u_mean;
    const float dz = rb.z_crown - lb.z_crown;
    meas.roll_angle = std::asin(std::max(-config_.max_cant_angle,
      std::min(config_.max_cant_angle, dz / std::max(1.0f, g_actual))));

    meas.confidence = 1.0f;
    last_valid_crown_z_ = meas.crown_z;
  }
  // Single-rail fallback when one side is occluded or in switch area
  else {
    bool has_l = false;
    float l_z = z_prior;
    float l_x = x_prior - half_g * cos_y;

    if (!left_bins.empty()) {
      auto best_it = std::max_element(left_bins.begin(), left_bins.end(),
        [](const RailBin & a, const RailBin & b) { return a.count < b.count; });
      if (best_it->count >= static_cast<size_t>(config_.min_rail_points) &&
          std::abs(best_it->z_crown - z_prior) <= 0.25f) {
        l_z = best_it->z_crown;
        l_x = best_it->x_mean;
        has_l = true;
      }
    }

    bool has_r = false;
    float r_z = z_prior;
    float r_x = x_prior + half_g * cos_y;

    if (!right_bins.empty()) {
      auto best_it = std::max_element(right_bins.begin(), right_bins.end(),
        [](const RailBin & a, const RailBin & b) { return a.count < b.count; });
      if (best_it->count >= static_cast<size_t>(config_.min_rail_points) &&
          std::abs(best_it->z_crown - z_prior) <= 0.25f) {
        r_z = best_it->z_crown;
        r_x = best_it->x_mean;
        has_r = true;
      }
    }

    if (has_l && !has_r) {
      meas.left_rail_x = l_x;
      meas.left_rail_z = l_z;
      meas.left_detected = true;
      meas.right_rail_x = l_x + nominal_g * cos_y;
      meas.right_rail_z = l_z;
      meas.crown_z = l_z;
      meas.track_center_x = l_x + half_g * cos_y;
      meas.confidence = 0.65f;
      last_valid_crown_z_ = meas.crown_z;
    } else if (has_r && !has_l) {
      meas.right_rail_x = r_x;
      meas.right_rail_z = r_z;
      meas.right_detected = true;
      meas.left_rail_x = r_x - nominal_g * cos_y;
      meas.left_rail_z = r_z;
      meas.crown_z = r_z;
      meas.track_center_x = r_x - half_g * cos_y;
      meas.confidence = 0.65f;
      last_valid_crown_z_ = meas.crown_z;
    } else if (!bed_zs.empty()) {
      // Both rails obscured: infer rail crown from central bed
      meas.crown_z = meas.bed_z + config_.bed_to_rail_height;
      meas.confidence = 0.40f;
    } else {
      // Extrapolate from previous valid rail crown
      meas.crown_z = last_valid_crown_z_;
      meas.confidence = 0.15f;
    }
  }

  return meas;
}

void DualRailTracker::smooth_profile(std::vector<FrenetWaypoint> & waypoints)
{
  if (waypoints.size() < 3) return;

  const float max_dz_ds = config_.max_grade_slope; // 0.035 (35 permille)

  // 1. Forward pass: enforce physical slope limit
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const float ds = waypoints[i].s - waypoints[i - 1].s;
    if (ds <= 0.001f) continue;

    const float max_delta_z = max_dz_ds * ds;
    const float dz = waypoints[i].z - waypoints[i - 1].z;
    const float clamped_dz = std::max(-max_delta_z, std::min(max_delta_z, dz));
    waypoints[i].z = waypoints[i - 1].z + clamped_dz;
  }

  // 2. Backward pass: enforce consistency from far horizon backwards
  for (size_t i = waypoints.size() - 1; i > 0; --i) {
    const float ds = waypoints[i].s - waypoints[i - 1].s;
    if (ds <= 0.001f) continue;

    const float max_delta_z = max_dz_ds * ds;
    const float dz = waypoints[i - 1].z - waypoints[i].z;
    const float clamped_dz = std::max(-max_delta_z, std::min(max_delta_z, dz));
    waypoints[i - 1].z = waypoints[i].z + clamped_dz;
  }

  // 3. 3-point Gaussian kernel smoothing [0.25, 0.50, 0.25]
  std::vector<float> smoothed_z(waypoints.size());
  smoothed_z.front() = waypoints.front().z;
  smoothed_z.back() = waypoints.back().z;

  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    smoothed_z[i] = 0.25f * waypoints[i - 1].z + 0.50f * waypoints[i].z + 0.25f * waypoints[i + 1].z;
  }

  for (size_t i = 0; i < waypoints.size(); ++i) {
    waypoints[i].z = smoothed_z[i];
  }
}

} // namespace metro_obstacle_detector
