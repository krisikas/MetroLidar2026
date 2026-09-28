#include "metro_obstacle_detector/track_geometry_spline.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace metro_obstacle_detector
{

TrackGeometrySpline::TrackGeometrySpline(const SplineTrackerConfig & config)
: config_(config)
{
}

void TrackGeometrySpline::set_config(const SplineTrackerConfig & config)
{
  config_ = config;
}

void TrackGeometrySpline::build_trajectory(
  const std::vector<Point3D> & points,
  const std::vector<DualRailMeasurement> & /* rail_measurements */,
  std::vector<FrenetWaypoint> & out_waypoints)
{
  out_waypoints.clear();
  if (points.empty()) return;

  const float lookahead = std::max(60.0f, config_.lookahead_distance);
  const float min_dist = config_.min_distance;

  // 1. O(N) Longitudinal bucketing along forward distance (-y)
  const float bucket_step = 2.0f;
  const int n_buckets = static_cast<int>(std::ceil((lookahead + 15.0f) / bucket_step)) + 1;
  std::vector<std::vector<const Point3D *>> buckets(n_buckets);
  for (int b = 0; b < n_buckets; ++b) {
    buckets[b].reserve(512);
  }

  for (const auto & p : points) {
    const float d = -p.y;
    if (d >= 0.0f && d < (lookahead + 12.0f)) {
      const int b = static_cast<int>(d / bucket_step);
      if (b >= 0 && b < n_buckets) {
        buckets[b].push_back(&p);
      }
    }
  }

  // 2. Dynamic near-field rail crown Z estimation (supports coupler to roof mounts)
  float curr_z = -1.15f;
  std::vector<float> near_zs;
  near_zs.reserve(1024);
  const int near_b_max = std::min(6, n_buckets - 1);
  for (int b = 0; b <= near_b_max; ++b) {
    for (const auto * p : buckets[b]) {
      const float d = -p->y;
      if (d >= 3.5f && d <= 12.0f && std::abs(p->x) <= 1.0f && p->z < -0.6f) {
        near_zs.push_back(p->z);
      }
    }
  }
  if (near_zs.size() >= 20) {
    std::sort(near_zs.begin(), near_zs.end());
    curr_z = near_zs[static_cast<size_t>(near_zs.size() * 0.90f)];
  }

  // Near-field lateral span to detect double-track tunnels vs single tubes
  std::vector<float> near_xs;
  near_xs.reserve(1024);
  for (int b = 0; b <= near_b_max; ++b) {
    for (const auto * p : buckets[b]) {
      const float d = -p->y;
      if (d >= 4.0f && d <= 16.0f && p->z >= -0.2f && p->z <= 2.2f) {
        near_xs.push_back(p->x);
      }
    }
  }
  bool is_double_track = false;
  if (near_xs.size() >= 50) {
    std::sort(near_xs.begin(), near_xs.end());
    const float p02 = near_xs[static_cast<size_t>(near_xs.size() * 0.02f)];
    const float p98 = near_xs[static_cast<size_t>(near_xs.size() * 0.98f)];
    if ((p98 - p02) > 6.5f) {
      is_double_track = true;
    }
  }

  float curr_x = 0.0f;
  float curr_y = -min_dist;
  float heading = 0.0f;
  float dz_ds = 0.0f;
  float curvature = 0.0f;
  float nominal_half_w = 2.15f;
  const float half_gauge = 0.760f; // Standard Russian gauge 1520 mm / 2
  const float min_rad = std::max(100.0f, config_.min_curve_radius);
  const float max_curv = 1.0f / min_rad;

  float dist = min_dist;
  out_waypoints.reserve(64);

  while (dist < lookahead) {
    const float ds = (dist < 40.0f) ? 2.0f : ((dist < 80.0f) ? 4.0f : 6.0f);
    const float actual_ds = std::min(ds, lookahead - dist);
    if (actual_ds < 1.0f && !out_waypoints.empty()) break;

    // Kinematics prediction along current curvature
    const float pred_heading = heading + curvature * (0.5f * actual_ds);
    const float pred_x = curr_x + std::sin(pred_heading) * (0.5f * actual_ds);
    const float pred_y = curr_y - std::cos(pred_heading) * (0.5f * actual_ds);
    const float pred_z = curr_z + dz_ds * (0.5f * actual_ds);

    const float cos_h = std::cos(pred_heading);
    const float sin_h = std::sin(pred_heading);

    // Query slice points from buckets
    const float d_center = -pred_y;
    const int b_min = std::max(0, static_cast<int>((d_center - actual_ds - 3.0f) / bucket_step));
    const int b_max = std::min(n_buckets - 1, static_cast<int>((d_center + actual_ds + 3.0f) / bucket_step));

    std::vector<float> left_rail_zs;
    std::vector<float> right_rail_zs;
    std::vector<float> bed_zs;
    std::vector<float> left_walls;
    std::vector<float> right_walls;

    left_rail_zs.reserve(64);
    right_rail_zs.reserve(64);
    bed_zs.reserve(128);
    left_walls.reserve(128);
    right_walls.reserve(128);

    const float half_step = 0.5f * actual_ds;
    for (int b = b_min; b <= b_max; ++b) {
      for (const auto * p : buckets[b]) {
        const float dx = p->x - pred_x;
        const float dy = p->y - pred_y;
        const float v_tan = dx * sin_h - dy * cos_h;
        if (std::abs(v_tan) <= half_step) {
          const float u = dx * cos_h + dy * sin_h;
          const float pz = p->z;

          // Rail heads and central trackbed
          if (pz >= pred_z - 0.45f && pz <= pred_z + 0.25f) {
            if (std::abs(u) <= 0.40f) {
              bed_zs.push_back(pz);
            }
            if (std::abs(u - (-half_gauge)) <= 0.14f) {
              left_rail_zs.push_back(pz);
            } else if (std::abs(u - half_gauge) <= 0.14f) {
              right_rail_zs.push_back(pz);
            }
          }

          // Tunnel walls: vertical span above floor
          if (pz >= pred_z + 0.50f && pz <= pred_z + 3.50f) {
            if (u >= -4.5f && u <= -1.60f) {
              left_walls.push_back(u);
            } else if (u >= 1.60f && u <= 4.5f) {
              right_walls.push_back(u);
            }
          }
        }
      }
    }

    // 1. Z estimation (Rail crown 90th percentile)
    float meas_z = pred_z;
    if (left_rail_zs.size() >= 3 && right_rail_zs.size() >= 3) {
      std::sort(left_rail_zs.begin(), left_rail_zs.end());
      std::sort(right_rail_zs.begin(), right_rail_zs.end());
      const float zl = left_rail_zs[static_cast<size_t>(left_rail_zs.size() * 0.90f)];
      const float zr = right_rail_zs[static_cast<size_t>(right_rail_zs.size() * 0.90f)];
      meas_z = std::max(zl, zr);
    } else if (left_rail_zs.size() >= 3) {
      std::sort(left_rail_zs.begin(), left_rail_zs.end());
      meas_z = left_rail_zs[static_cast<size_t>(left_rail_zs.size() * 0.90f)];
    } else if (right_rail_zs.size() >= 3) {
      std::sort(right_rail_zs.begin(), right_rail_zs.end());
      meas_z = right_rail_zs[static_cast<size_t>(right_rail_zs.size() * 0.90f)];
    } else if (bed_zs.size() >= 3) {
      std::sort(bed_zs.begin(), bed_zs.end());
      meas_z = bed_zs[bed_zs.size() / 2] + 0.18f;
    }

    const float target_dz = (meas_z - curr_z) / actual_ds;
    const float max_slope = 0.035f; // 35 permille railway standard
    dz_ds = std::clamp(target_dz, -max_slope, max_slope);
    const float updated_z = curr_z + dz_ds * actual_ds;

    // 2. Lateral measurement from Tunnel Walls
    const bool has_l = left_walls.size() >= 3;
    const bool has_r = right_walls.size() >= 3;
    float l_bound = -nominal_half_w;
    float r_bound = nominal_half_w;

    if (has_l) {
      std::sort(left_walls.begin(), left_walls.end());
      l_bound = left_walls[static_cast<size_t>(left_walls.size() * 0.90f)];
    }
    if (has_r) {
      std::sort(right_walls.begin(), right_walls.end());
      r_bound = right_walls[static_cast<size_t>(right_walls.size() * 0.10f)];
    }

    bool valid_wall = false;
    float meas_u = 0.0f;

    if (!is_double_track) {
      // In single-track tunnel: follow centerline of tube (tracks curves cleanly)
      if (has_l && has_r) {
        const float w = r_bound - l_bound;
        if (w >= 3.4f && w <= 5.6f) {
          meas_u = 0.5f * (l_bound + r_bound);
          valid_wall = true;
        }
      }
    }

    float updated_x = pred_x;
    float updated_y = pred_y;

    if (valid_wall) {
      const float range_conf = (dist < 50.0f) ? 1.0f : std::max(0.15f, 1.0f - (dist - 50.0f) / 100.0f);
      const float corr_gain = 0.45f * range_conf;
      updated_x = pred_x + corr_gain * meas_u * cos_h;
      updated_y = pred_y + corr_gain * meas_u * sin_h;

      const float dtheta = std::clamp((meas_u / 12.0f) * range_conf, -actual_ds / min_rad, actual_ds / min_rad);
      heading = std::clamp(pred_heading + dtheta, -0.45f, 0.45f);
      if (dist < 75.0f) {
        const float dkappa = (2.0f * meas_u / (28.0f * 28.0f)) * range_conf;
        curvature = std::clamp(curvature * 0.985f + dkappa, -max_curv, max_curv);
      } else {
        curvature *= 0.96f;
      }
    } else {
      updated_x = pred_x;
      updated_y = pred_y;
      heading = (is_double_track ? (heading * 0.85f) : (pred_heading * 0.85f));
      curvature *= 0.85f;
    }

    FrenetWaypoint wp;
    wp.s = dist - min_dist;
    wp.x = updated_x;
    wp.y = updated_y;
    wp.z = updated_z;
    wp.yaw = heading;
    wp.pitch = std::atan2(dz_ds, 1.0f);
    wp.roll = 0.0f;
    wp.curvature = curvature;
    wp.left_wall = -l_bound;
    wp.right_wall = r_bound;
    wp.confidence = valid_wall ? 1.0f : 0.6f;

    out_waypoints.push_back(wp);

    curr_x = updated_x + std::sin(heading) * (0.5f * actual_ds);
    curr_y = updated_y - std::cos(heading) * (0.5f * actual_ds);
    curr_z = updated_z + dz_ds * (0.5f * actual_ds);
    dist += actual_ds;
  }

  // Smooth elevation profile
  if (out_waypoints.size() >= 3) {
    for (size_t i = 1; i + 1 < out_waypoints.size(); ++i) {
      out_waypoints[i].z = 0.25f * out_waypoints[i - 1].z +
                           0.50f * out_waypoints[i].z +
                           0.25f * out_waypoints[i + 1].z;
    }
  }

  // Refine waypoint yaw and pitch from waypoint deltas
  for (size_t i = 0; i < out_waypoints.size(); ++i) {
    float next_x = out_waypoints[i].x;
    float next_y = out_waypoints[i].y;
    float next_z = out_waypoints[i].z;

    if (i + 1 < out_waypoints.size()) {
      next_x = out_waypoints[i + 1].x;
      next_y = out_waypoints[i + 1].y;
      next_z = out_waypoints[i + 1].z;
    } else if (i > 0) {
      next_x = out_waypoints[i].x + (out_waypoints[i].x - out_waypoints[i - 1].x);
      next_y = out_waypoints[i].y + (out_waypoints[i].y - out_waypoints[i - 1].y);
      next_z = out_waypoints[i].z + (out_waypoints[i].z - out_waypoints[i - 1].z);
    }

    const float dx = next_x - out_waypoints[i].x;
    const float dy = next_y - out_waypoints[i].y;
    const float dz = next_z - out_waypoints[i].z;

    out_waypoints[i].yaw = std::atan2(dx, -dy);
    out_waypoints[i].pitch = std::atan2(dz, -dy);
  }
}

bool TrackGeometrySpline::project_to_frenet(
  float px, float py, float pz,
  const std::vector<FrenetWaypoint> & waypoints,
  float & out_s, float & out_u, float & out_v,
  size_t & out_closest_idx) const
{
  if (waypoints.size() < 2) return false;

  // Find closest waypoint along Y
  size_t best_idx = 0;
  float min_dy = 1e6f;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const float dy = std::abs(waypoints[i].y - py);
    if (dy < min_dy) {
      min_dy = dy;
      best_idx = i;
    }
  }

  out_closest_idx = best_idx;
  const auto & wp = waypoints[best_idx];

  const float cos_yaw = std::cos(wp.yaw);
  const float sin_yaw = std::sin(wp.yaw);
  const float dx = px - wp.x;
  const float dy = py - wp.y;

  out_u = dx * cos_yaw + dy * sin_yaw; // Lateral offset
  out_v = pz - wp.z;                  // Height above rail crown
  out_s = wp.s + (dx * sin_yaw - dy * cos_yaw); // Longitudinal distance

  return true;
}

} // namespace metro_obstacle_detector
