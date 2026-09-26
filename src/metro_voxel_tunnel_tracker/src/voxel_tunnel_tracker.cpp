#include "metro_voxel_tunnel_tracker/voxel_tunnel_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <queue>

namespace metro_voxel_tunnel_tracker
{

VoxelTunnelTracker::VoxelTunnelTracker(const VoxelTrackerConfig & config)
: config_(config),
  voxel_grid_(config.voxel_config)
{
  init_slice_steps();
}

void VoxelTunnelTracker::set_config(const VoxelTrackerConfig & config)
{
  config_ = config;
  voxel_grid_.set_config(config.voxel_config);
  init_slice_steps();
}

void VoxelTunnelTracker::init_slice_steps()
{
  slice_ds_.clear();

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

  slice_voxels_.reserve(1024);
  rail_zs_.reserve(512);
  bed_zs_.reserve(1024);
  left_wall_us_.reserve(512);
  right_wall_us_.reserve(512);
  obstacle_voxel_ptrs_.reserve(2048);
}

void VoxelTunnelTracker::process(
  const std::vector<Point3D> & points,
  std::vector<TrackWaypoint> & out_trajectory,
  std::vector<ObstacleCluster> & out_obstacles)
{
  out_trajectory.clear();
  out_obstacles.clear();

  if (points.empty()) {
    return;
  }

  // 1. Вокселизация сырого облака точек
  voxel_grid_.insert_points(points);

  // 2. Трекинг 3D-траектории пути и габаритов тоннеля по вокселям
  track_trajectory(out_trajectory);

  // 3. Контроль свободности габарита (ГОСТ 9238) и кластеризация препятствий
  if (!out_trajectory.empty()) {
    extract_obstacles(out_trajectory, out_obstacles);
  }
}

void VoxelTunnelTracker::track_trajectory(std::vector<TrackWaypoint> & trajectory)
{
  trajectory.clear();
  if (num_slices_ <= 0 || voxel_grid_.size() == 0) {
    return;
  }

  trajectory.reserve(num_slices_);

  float curr_x = 0.0f;
  float curr_y = -config_.min_distance;
  float curr_z = config_.default_rail_z;
  float heading = 0.0f;
  float dz_ds = 0.0f;
  float curvature = 0.0f;
  float nominal_half_width = config_.single_tunnel_radius;

  const float half_gauge = 0.5f * config_.gauge;

  float dist = config_.min_distance;
  for (int i = 0; i < num_slices_; ++i) {
    const float actual_ds = slice_ds_[i];
    const float next_dist = dist + actual_ds;
    const float dist_ahead = dist + 0.5f * actual_ds;

    const float curv_scale = (dist_ahead < config_.curvature_freeze_dist) ? 1.0f :
      std::max(0.0f, 1.0f - (dist_ahead - config_.curvature_freeze_dist) / 35.0f);
    const float eff_curv = curvature * curv_scale;

    const float half_step = 0.5f * actual_ds;
    const float pred_heading = heading + eff_curv * half_step;
    const float pred_x = curr_x + std::sin(pred_heading) * half_step;
    const float pred_y = curr_y - std::cos(pred_heading) * half_step;
    const float pred_z = curr_z + dz_ds * half_step;

    const float cos_yaw = std::cos(pred_heading);
    const float sin_yaw = std::sin(pred_heading);

    // Запрос вокселей в окрестности текущего среза Френе
    voxel_grid_.query_frenet_slice(
      pred_x, pred_y, pred_heading,
      half_step, config_.wall_search_max_dist + 0.5f,
      slice_voxels_);

    rail_zs_.clear();
    bed_zs_.clear();
    left_wall_us_.clear();
    right_wall_us_.clear();

    float max_z_observed = pred_z + 2.5f;

    for (const auto * v_ptr : slice_voxels_) {
      const float dx = v_ptr->x - pred_x;
      const float dy = v_ptr->y - pred_y;
      const float u = dx * cos_yaw + dy * sin_yaw;
      const float pz = v_ptr->z;

      // Ходовое полотно и рельсы
      if (std::abs(u) <= config_.track_corridor_half_width) {
        if (pz >= pred_z + config_.rail_z_min_offset &&
            pz <= pred_z + config_.rail_z_max_offset)
        {
          bed_zs_.push_back(pz);
          if (std::abs(std::abs(u) - half_gauge) <= config_.rail_search_tolerance) {
            rail_zs_.push_back(pz);
          }
        }
      }

      // Стены и свод тоннеля (инвариантно к форме тоннеля)
      if (pz >= pred_z + config_.wall_z_min && pz <= pred_z + config_.wall_z_max) {
        if (std::abs(u) <= 2.5f && pz > max_z_observed) {
          max_z_observed = pz;
        }
        if (u <= -config_.wall_search_min_dist && u >= -config_.wall_search_max_dist) {
          left_wall_us_.push_back(u);
        } else if (u >= config_.wall_search_min_dist && u <= config_.wall_search_max_dist) {
          right_wall_us_.push_back(u);
        }
      }
    }

    // 1. Оценка отметки УГР (Z) с контролем предельного уклона ПТЭ
    float measured_z = pred_z;
    if (static_cast<int>(rail_zs_.size()) >= config_.min_rail_voxels) {
      const size_t idx = static_cast<size_t>(rail_zs_.size() * config_.rail_head_quantile);
      auto it = rail_zs_.begin() + idx;
      std::nth_element(rail_zs_.begin(), it, rail_zs_.end());
      measured_z = *it;
    } else if (static_cast<int>(bed_zs_.size()) >= config_.min_rail_voxels) {
      const size_t idx = static_cast<size_t>(bed_zs_.size() * config_.track_bed_quantile);
      auto it = bed_zs_.begin() + idx;
      std::nth_element(bed_zs_.begin(), it, bed_zs_.end());
      measured_z = *it + config_.rail_height_over_bed;
    }

    const float target_dz = (measured_z - curr_z) / actual_ds;
    dz_ds = std::clamp(target_dz, -config_.max_grade_slope, config_.max_grade_slope);
    const float updated_z = curr_z + dz_ds * actual_ds;

    // 2. Универсальная оценка границ стен без предположений о форме тоннеля
    const size_t min_wall_cnt = (dist_ahead < config_.wall_near_threshold) ?
      static_cast<size_t>(config_.min_wall_voxels_near) : static_cast<size_t>(config_.min_wall_voxels_far);
    const bool has_left = left_wall_us_.size() >= min_wall_cnt;
    const bool has_right = right_wall_us_.size() >= min_wall_cnt;

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
        // Асимметричный участок (платформа, двухпутный переход, стрелочный перевод)
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
    const float range_conf = (dist_ahead < 50.0f) ? 1.0f :
      std::max(0.15f, 1.0f - (dist_ahead - 50.0f) / 100.0f);
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
        curvature *= 0.96f;
      }
    } else {
      heading = std::clamp(heading, -config_.max_heading_slope, config_.max_heading_slope);
      curvature *= 0.75f;
    }

    // Границы габаритного коридора тоннеля
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
    trajectory.push_back(wp);

    curr_x = updated_x + std::sin(heading) * half_step;
    curr_y = updated_y - std::cos(heading) * half_step;
    curr_z = updated_z + dz_ds * half_step;
    dist = next_dist;
  }

  // Вертикальное сглаживание профиля пути
  if (trajectory.size() >= 3) {
    for (size_t k = 1; k < trajectory.size() - 1; ++k) {
      trajectory[k].z_rail = 0.25f * trajectory[k - 1].z_rail +
                             0.50f * trajectory[k].z_rail +
                             0.25f * trajectory[k + 1].z_rail;
    }
  }
}

void VoxelTunnelTracker::extract_obstacles(
  const std::vector<TrackWaypoint> & trajectory,
  std::vector<ObstacleCluster> & obstacles)
{
  obstacles.clear();
  obstacle_voxel_ptrs_.clear();

  if (trajectory.size() < 2) {
    return;
  }

  const auto & all_voxels = voxel_grid_.get_voxels();
  if (all_voxels.empty()) {
    return;
  }

  const float traj_y_start = trajectory.front().y;
  const float traj_y_end = trajectory.back().y;

  // 1. Поиск вокселей, вторгающихся в кинематический габарит вагона ГОСТ 9238
  for (const auto & v : all_voxels) {
    if (v.y > traj_y_start + 1.0f || v.y < traj_y_end - 1.0f) {
      continue;
    }

    // Поиск ближайшего вейпоинта пути по оси Y
    size_t closest_idx = 0;
    float min_dist_y = 1e6f;
    for (size_t i = 0; i < trajectory.size(); ++i) {
      const float dy = std::abs(trajectory[i].y - v.y);
      if (dy < min_dist_y) {
        min_dist_y = dy;
        closest_idx = i;
      }
    }

    const auto & wp = trajectory[closest_idx];
    const float delta_z = v.z - wp.z_rail;

    // Исключение рельсов, настила пути и пространства выше крыши поезда
    if (delta_z < config_.rail_head_clearance || delta_z > config_.carriage_height) {
      continue;
    }

    // Проекция вокселя на поперечную нормаль пути в вейпоинте
    const float cos_yaw = std::cos(wp.yaw);
    const float sin_yaw = std::sin(wp.yaw);
    const float dx = v.x - wp.x;
    const float dy = v.y - wp.y;
    const float u_lat = dx * cos_yaw + dy * sin_yaw;

    // Ступенчатый кинематический габарит вагона (ГОСТ 9238):
    // 1. Подвагонная зона (до контактного рельса): w = 1.15 м
    // 2. Зона платформы: w = 1.33 м
    // 3. Зона кузова вагона: w = 1.37 м
    // 4. Крышевой скат: сужение от 1.37 до 0.85 м
    float allowed_half_w = config_.waist_half_width;
    if (delta_z <= config_.contact_rail_height) {
      allowed_half_w = config_.undercarriage_half_width;
    } else if (delta_z <= config_.platform_height) {
      allowed_half_w = config_.platform_clearance_half_width;
    } else if (delta_z >= config_.carriage_wall_height) {
      const float t = (delta_z - config_.carriage_wall_height) /
        std::max(0.01f, config_.carriage_height - config_.carriage_wall_height);
      allowed_half_w = config_.waist_half_width + t * (config_.roof_half_width - config_.waist_half_width);
    }

    if (std::abs(u_lat) <= allowed_half_w) {
      obstacle_voxel_ptrs_.push_back(&v);
    }
  }

  if (obstacle_voxel_ptrs_.empty()) {
    return;
  }

  // 2. Кластеризация вокселей препятствий (Connected Component Clustering в 3D)
  const float dist_sq_thresh = config_.cluster_distance_thresh * config_.cluster_distance_thresh;
  const size_t n_obs = obstacle_voxel_ptrs_.size();
  std::vector<bool> visited(n_obs, false);

  for (size_t i = 0; i < n_obs; ++i) {
    if (visited[i]) continue;

    std::vector<const Voxel *> cluster_voxels;
    std::queue<size_t> q;

    visited[i] = true;
    q.push(i);

    while (!q.empty()) {
      size_t curr = q.front();
      q.pop();
      cluster_voxels.push_back(obstacle_voxel_ptrs_[curr]);

      const auto * v_curr = obstacle_voxel_ptrs_[curr];

      for (size_t j = 0; j < n_obs; ++j) {
        if (!visited[j]) {
          const auto * v_other = obstacle_voxel_ptrs_[j];
          const float ddx = v_curr->x - v_other->x;
          const float ddy = v_curr->y - v_other->y;
          const float ddz = v_curr->z - v_other->z;
          if (ddx * ddx + ddy * ddy + ddz * ddz <= dist_sq_thresh) {
            visited[j] = true;
            q.push(j);
          }
        }
      }
    }

    if (static_cast<int>(cluster_voxels.size()) >= config_.min_cluster_voxels) {
      float min_x = 1e6f, max_x = -1e6f;
      float min_y = 1e6f, max_y = -1e6f;
      float min_z = 1e6f, max_z = -1e6f;
      double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
      size_t total_points = 0;

      for (const auto * v : cluster_voxels) {
        min_x = std::min(min_x, v->x);
        max_x = std::max(max_x, v->x);
        min_y = std::min(min_y, v->y);
        max_y = std::max(max_y, v->y);
        min_z = std::min(min_z, v->z);
        max_z = std::max(max_z, v->z);
        sum_x += v->x;
        sum_y += v->y;
        sum_z += v->z;
        total_points += v->point_count;
      }

      const double inv_n = 1.0 / static_cast<double>(cluster_voxels.size());
      ObstacleCluster obs;
      obs.center_x = static_cast<float>(sum_x * inv_n);
      obs.center_y = static_cast<float>(sum_y * inv_n);
      obs.center_z = static_cast<float>(sum_z * inv_n);
      obs.size_x = std::max(0.15f, max_x - min_x + config_.voxel_config.voxel_size_x);
      obs.size_y = std::max(0.20f, max_y - min_y + config_.voxel_config.voxel_size_y);
      obs.size_z = std::max(0.10f, max_z - min_z + config_.voxel_config.voxel_size_z);
      obs.distance = -obs.center_y;
      obs.voxel_count = cluster_voxels.size();
      obs.total_points = total_points;

      obstacles.push_back(obs);
    }
  }

  // Сортировка препятствий по дальности (от ближайшего к дальнему)
  std::sort(obstacles.begin(), obstacles.end(), [](const ObstacleCluster & a, const ObstacleCluster & b) {
    return a.distance < b.distance;
  });
}

}  // namespace metro_voxel_tunnel_tracker
