#include "metro_tunnel_tracker/rail_geometry_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace metro_tunnel_tracker
{

RailGeometryTracker::RailGeometryTracker(const TrackerConfig & config)
: config_(config)
{
  init_buffers();
}

void RailGeometryTracker::set_config(const TrackerConfig & config)
{
  config_ = config;
  init_buffers();
}

void RailGeometryTracker::init_buffers()
{
  const float span_y = config_.lookahead_distance - config_.min_distance;
  const int num_slices = static_cast<int>(std::ceil(span_y / config_.slice_step));
  if (num_slices <= 0) {
    slice_ptrs_.clear();
    slice_y_centers_.clear();
    return;
  }

  slice_ptrs_.resize(num_slices);
  slice_y_centers_.resize(num_slices);

  for (int i = 0; i < num_slices; ++i) {
    const float y_start = -config_.min_distance - static_cast<float>(i) * config_.slice_step;
    slice_y_centers_[i] = y_start - 0.5f * config_.slice_step;
    slice_ptrs_[i].reserve(4096);
  }

  rail_head_zs_.reserve(2048);
  track_bed_zs_.reserve(4096);
  left_wall_xs_.reserve(2048);
  right_wall_xs_.reserve(2048);
  trough_xs_.reserve(1024);
  trajectory_buf_.reserve(num_slices);
}

std::vector<TrackWaypoint> RailGeometryTracker::estimate_track_trajectory(
  const std::vector<Point3D> & points)
{
  trajectory_buf_.clear();
  if (points.empty() || slice_ptrs_.empty()) {
    return trajectory_buf_;
  }

  const int num_slices = static_cast<int>(slice_ptrs_.size());
  for (int i = 0; i < num_slices; ++i) {
    slice_ptrs_[i].clear();
  }

  // Fast pointer-based binning: zero copy of 3D point data
  for (const auto & pt : points) {
    if (pt.y > -config_.min_distance || pt.y < -config_.lookahead_distance) {
      continue;
    }
    const int idx = static_cast<int>((-config_.min_distance - pt.y) / config_.slice_step);
    if (idx >= 0 && idx < num_slices) {
      slice_ptrs_[idx].push_back(&pt);
    }
  }

  float curr_x = 0.0f;
  float curr_z = config_.default_rail_z;
  float dx_dy = 0.0f;
  float dz_dy = 0.0f;
  float curvature = 0.0f;

  // Dynamic corridor half-width (adapts to round, rectangular, arched, or station corridors)
  float nominal_half_width = config_.single_tunnel_radius;

  for (int i = 0; i < num_slices; ++i) {
    const float dy = config_.slice_step;
    const float dist_ahead = -slice_y_centers_[i];

    const float x_pred = curr_x + dx_dy * dy + 0.5f * curvature * (dy * dy);
    const float z_pred = curr_z + dz_dy * dy;

    // Neighbor pointers for density accumulation at distances >= 45m
    const std::vector<const Point3D *> * prev_ptrs = (dist_ahead >= 45.0f && i > 0) ? &slice_ptrs_[i - 1] : nullptr;
    const std::vector<const Point3D *> * next_ptrs = (dist_ahead >= 45.0f && i + 1 < num_slices) ? &slice_ptrs_[i + 1] : nullptr;

    const float prev_z = curr_z;
    const float z_rail = estimate_slice_z(
      slice_ptrs_[i], prev_ptrs, next_ptrs, prev_z, x_pred, z_pred, dz_dy);

    float left_bound = x_pred - nominal_half_width;
    float right_bound = x_pred + nominal_half_width;
    float ceiling_z = 2.6f;
    float confidence = 1.0f;

    curr_x = estimate_slice_x(
      slice_ptrs_[i], prev_ptrs, next_ptrs, curr_x, z_rail, dist_ahead,
      dx_dy, curvature, nominal_half_width, left_bound, right_bound,
      ceiling_z, confidence);
    curr_z = z_rail;

    float corridor_l = curr_x - config_.clearance_corridor_half_width;
    float corridor_r = curr_x + config_.clearance_corridor_half_width;
    if (left_bound > corridor_l && (curr_x - left_bound) > 1.2f) {
      corridor_l = left_bound;
    }
    if (right_bound < corridor_r && (right_bound - curr_x) > 1.2f) {
      corridor_r = right_bound;
    }

    TrackWaypoint wp;
    wp.x = curr_x;
    wp.y = slice_y_centers_[i];
    wp.z_rail = curr_z;
    wp.left_boundary = corridor_l;
    wp.right_boundary = corridor_r;
    wp.ceiling_z = ceiling_z;
    wp.confidence = confidence;
    wp.valid = true;
    trajectory_buf_.push_back(wp);
  }

  // 3-point smoothing filter on vertical grade (Z) only.
  // We deliberately do NOT apply chord-cutting smoothing to X so curve arcs are preserved intact.
  if (trajectory_buf_.size() >= 3) {
    for (size_t i = 1; i < trajectory_buf_.size() - 1; ++i) {
      trajectory_buf_[i].z_rail = 0.25f * trajectory_buf_[i - 1].z_rail +
                                  0.50f * trajectory_buf_[i].z_rail +
                                  0.25f * trajectory_buf_[i + 1].z_rail;
    }
  }

  // Calculate tangent yaw and pitch
  for (size_t i = 0; i < trajectory_buf_.size(); ++i) {
    float next_x = trajectory_buf_[i].x;
    float next_y = trajectory_buf_[i].y - config_.slice_step;
    float next_z = trajectory_buf_[i].z_rail;

    if (i + 1 < trajectory_buf_.size()) {
      next_x = trajectory_buf_[i + 1].x;
      next_y = trajectory_buf_[i + 1].y;
      next_z = trajectory_buf_[i + 1].z_rail;
    }

    const float delta_x = next_x - trajectory_buf_[i].x;
    const float delta_y = next_y - trajectory_buf_[i].y;
    const float delta_z = next_z - trajectory_buf_[i].z_rail;

    trajectory_buf_[i].yaw = std::atan2(delta_x, -delta_y);
    trajectory_buf_[i].pitch = std::atan2(delta_z, -delta_y);
  }

  return trajectory_buf_;
}

float RailGeometryTracker::estimate_slice_z(
  const std::vector<const Point3D *> & slice_ptrs,
  const std::vector<const Point3D *> * prev_ptrs,
  const std::vector<const Point3D *> * next_ptrs,
  float prev_z,
  float x_pred,
  float z_pred,
  float & dz_dy)
{
  rail_head_zs_.clear();
  track_bed_zs_.clear();

  const float half_gauge = 0.5f * config_.gauge;
  const float rail_left_x = x_pred - half_gauge;
  const float rail_right_x = x_pred + half_gauge;
  const float rail_search_tol = 0.16f;

  auto collect_z = [&](const std::vector<const Point3D *> & pts) {
    for (const auto * pt : pts) {
      if (std::abs(pt->x - x_pred) <= config_.track_corridor_half_width) {
        if (pt->z <= z_pred + 0.30f && pt->z >= z_pred - 0.35f) {
          track_bed_zs_.push_back(pt->z);

          if (std::abs(pt->x - rail_left_x) <= rail_search_tol ||
              std::abs(pt->x - rail_right_x) <= rail_search_tol) {
            rail_head_zs_.push_back(pt->z);
          }
        }
      }
    }
  };

  collect_z(slice_ptrs);
  if (prev_ptrs) collect_z(*prev_ptrs);
  if (next_ptrs) collect_z(*next_ptrs);

  float measured_z = z_pred;
  if (rail_head_zs_.size() >= 3) {
    const size_t idx = static_cast<size_t>(rail_head_zs_.size() * 0.85f);
    auto it = rail_head_zs_.begin() + idx;
    std::nth_element(rail_head_zs_.begin(), it, rail_head_zs_.end());
    measured_z = *it;
  } else if (track_bed_zs_.size() >= 3) {
    const size_t idx = static_cast<size_t>(track_bed_zs_.size() * 0.85f);
    auto it = track_bed_zs_.begin() + idx;
    std::nth_element(track_bed_zs_.begin(), it, track_bed_zs_.end());
    measured_z = *it;
  }

  // Strictly clamp vertical slope to max_grade_slope (e.g. 0.035 m/m) so the track
  // cannot climb up pressure gates, ballast irregularities, or obstacles.
  const float target_slope = (measured_z - prev_z) / config_.slice_step;
  dz_dy = std::clamp(target_slope, -config_.max_grade_slope, config_.max_grade_slope);
  return prev_z + dz_dy * config_.slice_step;
}

float RailGeometryTracker::estimate_slice_x(
  const std::vector<const Point3D *> & slice_ptrs,
  const std::vector<const Point3D *> * prev_ptrs,
  const std::vector<const Point3D *> * next_ptrs,
  float prev_x,
  float z_rail,
  float dist_ahead,
  float & dx_dy,
  float & curvature,
  float & nominal_half_width,
  float & left_bound,
  float & right_bound,
  float & ceiling_z,
  float & confidence)
{
  const float dy = config_.slice_step;
  const float x_pred = prev_x + dx_dy * dy + 0.5f * curvature * (dy * dy);
  const float theta_pred = dx_dy + curvature * dy;

  left_wall_xs_.clear();
  right_wall_xs_.clear();

  float max_z_observed = z_rail + 2.5f;

  auto collect_x = [&](const std::vector<const Point3D *> & pts) {
    for (const auto * pt : pts) {
      if (pt->z >= z_rail + 0.35f && pt->z <= z_rail + 3.80f) {
        if (std::abs(pt->x - x_pred) <= 2.5f && pt->z > max_z_observed) {
          max_z_observed = pt->z;
        }
        if (pt->x <= x_pred - 1.25f && pt->x >= x_pred - 4.5f) {
          left_wall_xs_.push_back(pt->x);
        } else if (pt->x >= x_pred + 1.25f && pt->x <= x_pred + 4.5f) {
          right_wall_xs_.push_back(pt->x);
        }
      }
    }
  };

  collect_x(slice_ptrs);
  if (prev_ptrs) collect_x(*prev_ptrs);
  if (next_ptrs) collect_x(*next_ptrs);

  ceiling_z = max_z_observed - z_rail;

  const size_t min_wall_pts = (dist_ahead < 45.0f) ? 4 : ((dist_ahead < 85.0f) ? 2 : 1);
  const bool has_left = left_wall_xs_.size() >= min_wall_pts;
  const bool has_right = right_wall_xs_.size() >= min_wall_pts;

  float l_bound = x_pred - nominal_half_width;
  float r_bound = x_pred + nominal_half_width;

  // Quantile-based boundary estimation: extracts spatial surface of inner wall facing the track.
  // Completely immune to point-count imbalance (e.g. 10,000 points on outer wall vs 5 points on inner wall).
  if (has_left) {
    const size_t idx = static_cast<size_t>(left_wall_xs_.size() * 0.90f);
    auto it = left_wall_xs_.begin() + idx;
    std::nth_element(left_wall_xs_.begin(), it, left_wall_xs_.end());
    l_bound = *it;
  }

  if (has_right) {
    const size_t idx = static_cast<size_t>(right_wall_xs_.size() * 0.10f);
    auto it = right_wall_xs_.begin() + idx;
    std::nth_element(right_wall_xs_.begin(), it, right_wall_xs_.end());
    r_bound = *it;
  }

  left_bound = l_bound;
  right_bound = r_bound;

  float meas_x = x_pred;
  bool valid_meas = false;

  if (has_left && has_right) {
    const float obs_width = r_bound - l_bound;
    const float dist_l = x_pred - l_bound;
    const float dist_r = r_bound - x_pred;

    // Single symmetric corridor (round, square, arched): both walls roughly equidistant
    if (obs_width >= 3.2f && obs_width <= 5.4f && std::abs(dist_l - dist_r) <= 1.0f) {
      // Pure geometric midpoint between surfaces: 0% point density bias!
      meas_x = 0.5f * (l_bound + r_bound);
      valid_meas = true;
      if (dist_ahead < 50.0f) {
        nominal_half_width = 0.90f * nominal_half_width + 0.10f * (0.5f * obs_width);
      }
    } else {
      // Asymmetric corridor (double-track, station, switch): follow near continuous wall
      const float err_l = std::abs(dist_l - nominal_half_width);
      const float err_r = std::abs(dist_r - nominal_half_width);
      if (err_l < 0.65f && err_l <= err_r) {
        meas_x = l_bound + nominal_half_width;
        valid_meas = true;
      } else if (err_r < 0.65f && err_r < err_l) {
        meas_x = r_bound - nominal_half_width;
        valid_meas = true;
      }
    }
  } else if (has_right && !has_left) {
    // Single-wall tracking (crucial for curves where LiDAR shines directly onto outer wall,
    // while inner wall is shadowed or out of FOV).
    const float dist_r = r_bound - x_pred;
    if (std::abs(dist_r - nominal_half_width) < 0.85f) {
      meas_x = r_bound - nominal_half_width;
      valid_meas = true;
    }
  } else if (has_left && !has_right) {
    // Single-wall tracking for left-facing curves
    const float dist_l = x_pred - l_bound;
    if (std::abs(dist_l - nominal_half_width) < 0.85f) {
      meas_x = l_bound + nominal_half_width;
      valid_meas = true;
    }
  }

  const float max_curv = 1.0f / config_.min_curve_radius;
  const float max_dslope = dy / config_.min_curve_radius;
  const float range_conf = (dist_ahead < 60.0f) ? 1.0f :
                           std::max(0.20f, 1.0f - (dist_ahead - 60.0f) / 110.0f);
  confidence = valid_meas ? range_conf : (0.5f * range_conf);

  if (valid_meas) {
    // Critically damped observer: updates position, heading, and curvature without numerical differentiator ringing
    const float innov_x = meas_x - x_pred;
    const float K_x = 0.55f * range_conf;
    const float updated_x = x_pred + K_x * innov_x;

    // Heading innovation over a stable 12m spatial baseline, strictly clamped by min_curve_radius
    const float max_dtheta = dy / config_.min_curve_radius;
    float dtheta = (innov_x / 12.0f) * range_conf;
    dtheta = std::clamp(dtheta, -max_dtheta, max_dtheta);
    dx_dy = theta_pred + dtheta;
    dx_dy = std::clamp(dx_dy, -0.45f, 0.45f);

    // Curvature innovation over a stable 28m arc baseline
    const float dkappa = (2.0f * innov_x / (28.0f * 28.0f)) * range_conf;
    curvature = (curvature * 0.985f) + dkappa;
    curvature = std::clamp(curvature, -max_curv, max_curv);

    return updated_x;
  } else {
    // On unobserved / sparse slices: propagate smoothly along current curve without decaying heading to zero
    dx_dy = theta_pred;
    dx_dy = std::clamp(dx_dy, -0.45f, 0.45f);
    curvature *= 0.985f;
    return x_pred;
  }
}

}  // namespace metro_tunnel_tracker


