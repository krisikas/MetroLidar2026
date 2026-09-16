#include "metro_tunnel_tracker/rail_geometry_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace metro_tunnel_tracker
{

RailGeometryTracker::RailGeometryTracker(const TrackerConfig & config)
: config_(config)
{
}

std::vector<TrackWaypoint> RailGeometryTracker::estimate_track_trajectory(
  const std::vector<Point3D> & points)
{
  std::vector<TrackWaypoint> trajectory;
  if (points.empty()) {
    return trajectory;
  }

  const float span_y = config_.lookahead_distance - config_.min_distance;
  const int num_slices = static_cast<int>(std::ceil(span_y / config_.slice_step));
  if (num_slices <= 0) {
    return trajectory;
  }

  std::vector<std::vector<Point3D>> slices(num_slices);
  std::vector<float> slice_y_centers(num_slices);

  for (int i = 0; i < num_slices; ++i) {
    const float y_start = -config_.min_distance - static_cast<float>(i) * config_.slice_step;
    slice_y_centers[i] = y_start - 0.5f * config_.slice_step;
  }

  for (const auto & pt : points) {
    if (pt.y > -config_.min_distance || pt.y < -config_.lookahead_distance) {
      continue;
    }
    const int idx = static_cast<int>((-config_.min_distance - pt.y) / config_.slice_step);
    if (idx >= 0 && idx < num_slices) {
      slices[idx].push_back(pt);
    }
  }

  float curr_x = 0.0f;
  float curr_z = config_.default_rail_z;
  float dx_dy = 0.0f;
  float dz_dy = 0.0f;
  float curvature = 0.0f;

  trajectory.reserve(num_slices);

  for (int i = 0; i < num_slices; ++i) {
    const float dy = config_.slice_step;
    const float dist_ahead = -slice_y_centers[i];

    const float x_pred = curr_x + dx_dy * dy + 0.5f * curvature * (dy * dy);
    const float z_pred = curr_z + dz_dy * dy;

    // Aggregate points for distance-adaptive density
    std::vector<Point3D> active_pts = slices[i];
    if (dist_ahead >= 50.0f) {
      if (i > 0) {
        active_pts.insert(active_pts.end(), slices[i - 1].begin(), slices[i - 1].end());
      }
      if (i + 1 < num_slices) {
        active_pts.insert(active_pts.end(), slices[i + 1].begin(), slices[i + 1].end());
      }
    }

    const float z_rail = estimate_slice_z(active_pts, x_pred, z_pred, dz_dy);

    float left_bound = x_pred - config_.single_tunnel_radius;
    float right_bound = x_pred + config_.single_tunnel_radius;

    curr_x = estimate_slice_x(
      active_pts, curr_x, z_rail, dist_ahead, dx_dy, curvature, left_bound, right_bound);
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
    wp.y = slice_y_centers[i];
    wp.z_rail = curr_z;
    wp.left_boundary = corridor_l;
    wp.right_boundary = corridor_r;
    wp.valid = true;
    trajectory.push_back(wp);
  }

  if (trajectory.size() >= 3) {
    for (size_t i = 1; i < trajectory.size() - 1; ++i) {
      trajectory[i].x = 0.25f * trajectory[i - 1].x + 0.50f * trajectory[i].x + 0.25f * trajectory[i + 1].x;
      trajectory[i].z_rail = 0.25f * trajectory[i - 1].z_rail + 0.50f * trajectory[i].z_rail + 0.25f * trajectory[i + 1].z_rail;
    }
  }

  for (size_t i = 0; i < trajectory.size(); ++i) {
    float next_x = trajectory[i].x;
    float next_y = trajectory[i].y - config_.slice_step;
    float next_z = trajectory[i].z_rail;

    if (i + 1 < trajectory.size()) {
      next_x = trajectory[i + 1].x;
      next_y = trajectory[i + 1].y;
      next_z = trajectory[i + 1].z_rail;
    }

    const float delta_x = next_x - trajectory[i].x;
    const float delta_y = next_y - trajectory[i].y;
    const float delta_z = next_z - trajectory[i].z_rail;

    trajectory[i].yaw = std::atan2(delta_x, -delta_y);
    trajectory[i].pitch = std::atan2(delta_z, -delta_y);
  }

  return trajectory;
}

float RailGeometryTracker::estimate_slice_z(
  const std::vector<Point3D> & slice_points,
  float x_pred,
  float z_pred,
  float & dz_dy)
{
  std::vector<float> track_bed_zs;
  track_bed_zs.reserve(slice_points.size() / 4);

  for (const auto & pt : slice_points) {
    if (std::abs(pt.x - x_pred) <= config_.track_corridor_half_width) {
      if (pt.z <= z_pred + 0.50f && pt.z >= z_pred - 0.80f) {
        track_bed_zs.push_back(pt.z);
      }
    }
  }

  if (track_bed_zs.size() >= 3) {
    std::sort(track_bed_zs.begin(), track_bed_zs.end());
    const size_t q_idx = static_cast<size_t>(track_bed_zs.size() * 0.75f);
    const float measured_z = track_bed_zs[q_idx];

    const float updated_z = 0.60f * measured_z + 0.40f * z_pred;
    const float delta = (updated_z - (z_pred - dz_dy * config_.slice_step)) / config_.slice_step;
    dz_dy = std::clamp(delta, -config_.max_grade_slope, config_.max_grade_slope);
    return updated_z;
  }

  return z_pred;
}

float RailGeometryTracker::estimate_slice_x(
  const std::vector<Point3D> & slice_points,
  float prev_x,
  float z_rail,
  float dist_ahead,
  float & dx_dy,
  float & curvature,
  float & left_bound,
  float & right_bound)
{
  const float dy = config_.slice_step;
  const float x_pred = prev_x + dx_dy * dy + 0.5f * curvature * (dy * dy);

  std::vector<float> left_wall_xs;
  std::vector<float> right_wall_xs;

  for (const auto & pt : slice_points) {
    if (pt.z >= z_rail + 0.40f && pt.z <= z_rail + 2.40f) {
      if (pt.x < x_pred && pt.x >= x_pred - 5.5f) {
        left_wall_xs.push_back(pt.x);
      } else if (pt.x > x_pred && pt.x <= x_pred + 5.5f) {
        right_wall_xs.push_back(pt.x);
      }
    }
  }

  const size_t min_wall_pts = (dist_ahead < 50.0f) ? 4 : ((dist_ahead < 90.0f) ? 2 : 1);
  const bool has_left = left_wall_xs.size() >= min_wall_pts;
  const bool has_right = right_wall_xs.size() >= min_wall_pts;

  float l_bound = x_pred - config_.single_tunnel_radius;
  float r_bound = x_pred + config_.single_tunnel_radius;

  if (has_left) {
    std::sort(left_wall_xs.begin(), left_wall_xs.end());
    const size_t idx = static_cast<size_t>(left_wall_xs.size() * 0.90f);
    l_bound = left_wall_xs[idx];
  }

  if (has_right) {
    std::sort(right_wall_xs.begin(), right_wall_xs.end());
    const size_t idx = static_cast<size_t>(right_wall_xs.size() * 0.10f);
    r_bound = right_wall_xs[idx];
  }

  left_bound = l_bound;
  right_bound = r_bound;

  float meas_x = x_pred;
  bool valid_meas = false;

  const float dist_l = x_pred - l_bound;
  const float dist_r = r_bound - x_pred;

  if (has_left && has_right) {
    const float width = r_bound - l_bound;
    if (width >= 3.2f && width <= 5.2f && dist_l < 2.8f && dist_r < 2.8f) {
      meas_x = 0.5f * (l_bound + r_bound);
      valid_meas = true;
    } else if (dist_l < 2.8f && dist_r >= 2.8f) {
      meas_x = l_bound + config_.single_tunnel_radius;
      valid_meas = true;
    } else if (dist_r < 2.8f && dist_l >= 2.8f) {
      meas_x = r_bound - config_.single_tunnel_radius;
      valid_meas = true;
    }
  } else if (has_left && dist_l < 2.8f) {
    meas_x = l_bound + config_.single_tunnel_radius;
    valid_meas = true;
  } else if (has_right && dist_r < 2.8f) {
    meas_x = r_bound - config_.single_tunnel_radius;
    valid_meas = true;
  }

  std::vector<float> trough_xs;
  for (const auto & pt : slice_points) {
    if (std::abs(pt.x - x_pred) <= 0.50f && pt.z < z_rail - 0.15f) {
      trough_xs.push_back(pt.x);
    }
  }

  if (trough_xs.size() >= 5) {
    const float trough_center = std::accumulate(trough_xs.begin(), trough_xs.end(), 0.0f) / trough_xs.size();
    if (valid_meas) {
      meas_x = 0.60f * meas_x + 0.40f * trough_center;
    } else {
      meas_x = trough_center;
      valid_meas = true;
    }
  }

  const float max_dslope = dy / config_.min_curve_radius;

  if (valid_meas) {
    const float target_slope = (meas_x - prev_x) / dy;
    const float dslope = std::clamp(target_slope - dx_dy, -max_dslope, max_dslope);
    curvature = 0.75f * curvature + 0.25f * (dslope / dy);
    dx_dy += dslope;
    dx_dy = std::clamp(dx_dy, -0.45f, 0.45f);
  } else {
    curvature *= 0.98f;
    dx_dy += curvature * dy;
    dx_dy = std::clamp(dx_dy, -0.45f, 0.45f);
  }

  return prev_x + dx_dy * dy;
}

}  // namespace metro_tunnel_tracker


