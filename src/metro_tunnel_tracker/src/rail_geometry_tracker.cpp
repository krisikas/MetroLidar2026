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
  slice_ds_.clear();

  // Range-Adaptive Slicing zone boundaries along arc length:
  // Zone 0: [min_distance .. zone1_start] with step slice_step
  // Zone 1: [zone1_start  .. zone2_start] with step slice_step * 2
  // Zone 2: [zone2_start  .. lookahead]   with step slice_step * 4
  float dist = config_.min_distance;
  while (dist < config_.lookahead_distance) {
    float ds = config_.slice_step;
    if (dist >= config_.zone2_start) {
      ds = config_.slice_step * 4.0f;
    } else if (dist >= config_.zone1_start) {
      ds = config_.slice_step * 2.0f;
    }

    float next_dist = std::min(dist + ds, config_.lookahead_distance);
    float actual_ds = next_dist - dist;
    if (actual_ds < 0.5f * config_.slice_step && !slice_ds_.empty()) {
      slice_ds_.back() += actual_ds;
      break;
    }
    slice_ds_.push_back(actual_ds);
    dist = next_dist;
  }

  num_slices_ = static_cast<int>(slice_ds_.size());
  if (num_slices_ <= 0) {
    return;
  }

  // Pre-allocate spatial Y-buckets (2.0m bins along sensor forward axis)
  bucket_step_ = 2.0f;
  num_buckets_ = static_cast<int>(std::ceil((config_.lookahead_distance + 15.0f) / bucket_step_)) + 1;
  y_buckets_.resize(num_buckets_);
  for (int b = 0; b < num_buckets_; ++b) {
    y_buckets_[b].reserve(2048);
  }

  // Pre-allocate scratch buffers
  rail_head_zs_.reserve(2048);
  track_bed_zs_.reserve(4096);
  left_wall_us_.reserve(2048);
  right_wall_us_.reserve(2048);
  trajectory_buf_.reserve(num_slices_);
}

std::vector<TrackWaypoint> RailGeometryTracker::estimate_track_trajectory(
  const std::vector<Point3D> & points)
{
  trajectory_buf_.clear();
  if (points.empty() || slice_ds_.empty()) {
    return trajectory_buf_;
  }

  for (int b = 0; b < num_buckets_; ++b) {
    y_buckets_[b].clear();
  }

  // O(1) bucketing of raw point pointers by forward distance (-y)
  const float max_dist_limit = config_.lookahead_distance + 12.0f;
  for (const auto & pt : points) {
    const float d = -pt.y;
    if (d >= 0.0f && d < max_dist_limit && std::abs(pt.x) <= config_.max_lateral_offset) {
      const int b = static_cast<int>(d / bucket_step_);
      if (b >= 0 && b < num_buckets_) {
        y_buckets_[b].push_back(&pt);
      }
    }
  }

  // Sequential adaptive tracking along the curving trajectory (Frenet-Serret normal slicing)
  float curr_x = 0.0f;
  float curr_y = -config_.min_distance;
  float curr_z = config_.default_rail_z;
  float heading = 0.0f;
  float dz_ds = 0.0f;
  float curvature = 0.0f;
  float nominal_half_width = config_.single_tunnel_radius;

  float dist = config_.min_distance;
  for (int i = 0; i < num_slices_; ++i) {
    const float actual_ds = slice_ds_[i];
    const float next_dist = dist + actual_ds;
    const float dist_ahead = dist + 0.5f * actual_ds;

    // Curvature prediction influence dampened beyond curvature_freeze_dist
    const float curv_scale = (dist_ahead < config_.curvature_freeze_dist) ? 1.0f :
      std::max(0.0f, 1.0f - (dist_ahead - config_.curvature_freeze_dist) / 35.0f);
    const float eff_curv = curvature * curv_scale;

    // Predict waypoint at the center of the adaptive curved slice
    const float half_step = 0.5f * actual_ds;
    const float pred_heading = heading + eff_curv * half_step;
    const float pred_x = curr_x + std::sin(pred_heading) * half_step;
    const float pred_y = curr_y - std::cos(pred_heading) * half_step;
    const float pred_z = curr_z + dz_ds * half_step;

    const float cos_yaw = std::cos(pred_heading);
    const float sin_yaw = std::sin(pred_heading);

    // Retrieve candidate point buckets spanning this curved normal slice
    const float d_center = -pred_y;
    const int b_min = std::max(0, static_cast<int>((d_center - actual_ds - 3.0f) / bucket_step_));
    const int b_max = std::min(num_buckets_ - 1, static_cast<int>((d_center + actual_ds + 3.0f) / bucket_step_));

    const float half_gauge = 0.5f * config_.gauge;
    rail_head_zs_.clear();
    track_bed_zs_.clear();
    left_wall_us_.clear();
    right_wall_us_.clear();

    float max_z_observed = pred_z + 2.5f;

    // Project points into local Frenet-Serret coordinates:
    // v: along-track offset (slice normal condition: |v| <= actual_ds / 2)
    // u: cross-track offset (perpendicular to track)
    for (int b = b_min; b <= b_max; ++b) {
      for (const auto * pt : y_buckets_[b]) {
        const float dx = pt->x - pred_x;
        const float dy = pt->y - pred_y;
        const float v = dx * sin_yaw - dy * cos_yaw;

        if (std::abs(v) <= half_step) {
          const float u = dx * cos_yaw + dy * sin_yaw;

          // Rail head & track bed detection
          if (std::abs(u) <= config_.track_corridor_half_width) {
            if (pt->z >= pred_z + config_.rail_z_min_offset &&
                pt->z <= pred_z + config_.rail_z_max_offset)
            {
              track_bed_zs_.push_back(pt->z);
              if (std::abs(std::abs(u) - half_gauge) <= config_.rail_search_tolerance) {
                rail_head_zs_.push_back(pt->z);
              }
            }
          }

          // Tunnel walls & ceiling detection
          if (pt->z >= pred_z + config_.wall_z_min && pt->z <= pred_z + config_.wall_z_max) {
            if (std::abs(u) <= 2.5f && pt->z > max_z_observed) {
              max_z_observed = pt->z;
            }
            if (u <= -config_.wall_search_min_dist && u >= -config_.wall_search_max_dist) {
              left_wall_us_.push_back(u);
            } else if (u >= config_.wall_search_min_dist && u <= config_.wall_search_max_dist) {
              right_wall_us_.push_back(u);
            }
          }
        }
      }
    }

    // 1. Z estimation with strict slope clamping
    float measured_z = pred_z;
    if (static_cast<int>(rail_head_zs_.size()) >= config_.min_rail_points) {
      const size_t idx = static_cast<size_t>(rail_head_zs_.size() * config_.rail_head_quantile);
      auto it = rail_head_zs_.begin() + idx;
      std::nth_element(rail_head_zs_.begin(), it, rail_head_zs_.end());
      measured_z = *it;
    } else if (static_cast<int>(track_bed_zs_.size()) >= config_.min_rail_points) {
      const size_t idx = static_cast<size_t>(track_bed_zs_.size() * config_.track_bed_quantile);
      auto it = track_bed_zs_.begin() + idx;
      std::nth_element(track_bed_zs_.begin(), it, track_bed_zs_.end());
      measured_z = *it + config_.rail_height_over_bed;
    }

    const float target_dz = (measured_z - curr_z) / actual_ds;
    dz_ds = std::clamp(target_dz, -config_.max_grade_slope, config_.max_grade_slope);
    const float updated_z = curr_z + dz_ds * actual_ds;

    // 2. Wall & lateral trajectory estimation in normal coordinates
    const size_t min_wall_pts = (dist_ahead < config_.wall_near_threshold) ?
      static_cast<size_t>(config_.min_wall_points_near) : static_cast<size_t>(config_.min_wall_points_far);
    const bool has_left = left_wall_us_.size() >= min_wall_pts;
    const bool has_right = right_wall_us_.size() >= min_wall_pts;

    float l_bound_u = -nominal_half_width;
    float r_bound_u = nominal_half_width;

    if (has_left) {
      const size_t idx = static_cast<size_t>(left_wall_us_.size() * config_.left_wall_quantile);
      auto it = left_wall_us_.begin() + idx;
      std::nth_element(left_wall_us_.begin(), it, left_wall_us_.end());
      l_bound_u = *it;
    }
    if (has_right) {
      const size_t idx = static_cast<size_t>(right_wall_us_.size() * config_.right_wall_quantile);
      auto it = right_wall_us_.begin() + idx;
      std::nth_element(right_wall_us_.begin(), it, right_wall_us_.end());
      r_bound_u = *it;
    }

    float meas_u = 0.0f;
    bool valid_meas = false;

    if (has_left && has_right) {
      const float obs_width = r_bound_u - l_bound_u;
      const float dist_l = -l_bound_u;
      const float dist_r = r_bound_u;

      if (obs_width >= config_.symmetric_tunnel_min_width &&
          obs_width <= config_.symmetric_tunnel_max_width &&
          std::abs(dist_l - dist_r) <= config_.symmetric_tunnel_wall_tolerance)
      {
        meas_u = 0.5f * (l_bound_u + r_bound_u);
        valid_meas = true;
        if (dist_ahead < 50.0f) {
          nominal_half_width = 0.90f * nominal_half_width + 0.10f * (0.5f * obs_width);
        }
      } else {
        const float err_l = std::abs(dist_l - nominal_half_width);
        const float err_r = std::abs(dist_r - nominal_half_width);
        if (err_l < config_.wall_tracking_error_tolerance && err_l <= err_r) {
          meas_u = l_bound_u + nominal_half_width;
          valid_meas = true;
        } else if (err_r < config_.wall_tracking_error_tolerance) {
          meas_u = r_bound_u - nominal_half_width;
          valid_meas = true;
        }
      }
    } else if (has_right && !has_left) {
      const float dist_r = r_bound_u;
      if (dist_ahead < 15.0f) {
        nominal_half_width = 0.80f * nominal_half_width + 0.20f * dist_r;
      }
      if (std::abs(dist_r - nominal_half_width) < config_.single_wall_tolerance) {
        meas_u = r_bound_u - nominal_half_width;
        valid_meas = true;
      }
    } else if (has_left && !has_right) {
      const float dist_l = -l_bound_u;
      if (dist_ahead < 15.0f) {
        nominal_half_width = 0.80f * nominal_half_width + 0.20f * dist_l;
      }
      if (std::abs(dist_l - nominal_half_width) < config_.single_wall_tolerance) {
        meas_u = l_bound_u + nominal_half_width;
        valid_meas = true;
      }
    }

    const float max_curv = 1.0f / config_.min_curve_radius;
    const float max_dslope = actual_ds / config_.min_curve_radius;
    // Enhanced range confidence for long-distance stability (smooth decay up to 180m)
    const float range_conf = (dist_ahead < 60.0f) ? 1.0f :
      std::max(0.25f, 1.0f - (dist_ahead - 60.0f) / 130.0f);
    const float confidence = valid_meas ? range_conf : (0.5f * range_conf);

    float updated_x = pred_x;
    float updated_y = pred_y;

    if (valid_meas) {
      const float K_x = 0.55f * range_conf;
      const float corr_u = K_x * meas_u;
      updated_x = pred_x + corr_u * cos_yaw;
      updated_y = pred_y + corr_u * sin_yaw;

      float dtheta = (meas_u / 12.0f) * range_conf;
      dtheta = std::clamp(dtheta, -max_dslope, max_dslope);
      heading = std::clamp(pred_heading + dtheta, -config_.max_heading_slope, config_.max_heading_slope);

      if (dist_ahead < config_.curvature_freeze_dist) {
        const float dkappa = (2.0f * meas_u / (28.0f * 28.0f)) * range_conf;
        curvature = (curvature * 0.985f) + dkappa;
        curvature = std::clamp(curvature, -max_curv, max_curv);
      } else {
        // More gentle dampening beyond freeze distance to maintain curve continuation
        curvature *= 0.98f;
      }
    } else {
      heading = std::clamp(heading, -config_.max_heading_slope, config_.max_heading_slope);
      curvature *= 0.85f;
    }

    // Clearance and tunnel boundaries
    float corridor_l = updated_x - config_.clearance_corridor_half_width;
    float corridor_r = updated_x + config_.clearance_corridor_half_width;
    const float left_bound_x = updated_x + l_bound_u * cos_yaw;
    const float right_bound_x = updated_x + r_bound_u * cos_yaw;
    if (left_bound_x > corridor_l && (updated_x - left_bound_x) > 1.2f) {
      corridor_l = left_bound_x;
    }
    if (right_bound_x < corridor_r && (right_bound_x - updated_x) > 1.2f) {
      corridor_r = right_bound_x;
    }

    TrackWaypoint wp;
    wp.x = updated_x;
    wp.y = updated_y;
    wp.z_rail = updated_z;
    wp.yaw = heading;
    wp.pitch = std::atan2(dz_ds, 1.0f);
    wp.left_boundary = corridor_l;
    wp.right_boundary = corridor_r;
    wp.ceiling_z = max_z_observed - updated_z;
    wp.confidence = confidence;
    wp.valid = true;
    wp.curvature = curvature;
    wp.rail_points_count = static_cast<int>(rail_head_zs_.size() + track_bed_zs_.size());
    wp.has_left_wall = has_left;
    wp.has_right_wall = has_right;
    trajectory_buf_.push_back(wp);

    // Advance state to start of next slice
    curr_x = updated_x + std::sin(heading) * half_step;
    curr_y = updated_y - std::cos(heading) * half_step;
    curr_z = updated_z + dz_ds * half_step;
    dist = next_dist;
  }

  // 3-point smoothing filter on vertical grade (Z)
  if (trajectory_buf_.size() >= 3) {
    for (size_t i = 1; i < trajectory_buf_.size() - 1; ++i) {
      trajectory_buf_[i].z_rail = 0.25f * trajectory_buf_[i - 1].z_rail +
                                  0.50f * trajectory_buf_[i].z_rail +
                                  0.25f * trajectory_buf_[i + 1].z_rail;
    }
  }

  return trajectory_buf_;
}

}  // namespace metro_tunnel_tracker
