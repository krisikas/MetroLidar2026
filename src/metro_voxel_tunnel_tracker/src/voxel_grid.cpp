#include "metro_voxel_tunnel_tracker/voxel_grid.hpp"
#include <algorithm>
#include <cmath>

namespace metro_voxel_tunnel_tracker
{

SparseVoxelGrid::SparseVoxelGrid(const VoxelGridConfig & config)
{
  set_config(config);
  voxels_.reserve(65536);
  accumulators_.reserve(65536);
  key_to_index_.reserve(65536);
}

void SparseVoxelGrid::set_config(const VoxelGridConfig & config)
{
  config_ = config;
  inv_size_x_ = 1.0f / std::max(0.01f, config_.voxel_size_x);
  inv_size_y_ = 1.0f / std::max(0.01f, config_.voxel_size_y);
  inv_size_z_ = 1.0f / std::max(0.01f, config_.voxel_size_z);

  bucket_size_y_ = 2.0f;
  num_y_buckets_ = static_cast<int>(std::ceil((config_.max_forward_range + 15.0f) / bucket_size_y_)) + 1;
  y_bucket_indices_.resize(num_y_buckets_);
  for (int b = 0; b < num_y_buckets_; ++b) {
    y_bucket_indices_[b].reserve(2048);
  }
}

void SparseVoxelGrid::clear()
{
  key_to_index_.clear();
  voxels_.clear();
  accumulators_.clear();
  for (auto & bucket : y_bucket_indices_) {
    bucket.clear();
  }
}

void SparseVoxelGrid::insert_points(const std::vector<Point3D> & points)
{
  clear();
  if (points.empty()) {
    return;
  }

  const float min_y_limit = -config_.max_forward_range - 10.0f;
  const float max_y_limit = 1.0f;

  for (const auto & pt : points) {
    if (pt.y < min_y_limit || pt.y > max_y_limit) continue;
    if (std::abs(pt.x) > config_.max_lateral_range) continue;
    if (pt.z < config_.min_z_limit || pt.z > config_.max_z_limit) continue;

    const int32_t ix = to_ix(pt.x);
    const int32_t iy = to_iy(pt.y);
    const int32_t iz = to_iz(pt.z);
    const uint64_t key = encode_key(ix, iy, iz);

    auto it = key_to_index_.find(key);
    if (it == key_to_index_.end()) {
      const size_t new_idx = voxels_.size();
      key_to_index_[key] = new_idx;

      Voxel v;
      v.ix = ix;
      v.iy = iy;
      v.iz = iz;
      v.x = pt.x;
      v.y = pt.y;
      v.z = pt.z;
      v.min_z = pt.z;
      v.max_z = pt.z;
      v.mean_intensity = pt.intensity;
      v.point_count = 1;
      v.type = VoxelType::UNCLASSIFIED;
      voxels_.push_back(v);

      VoxelAccumulator acc;
      acc.sum_x = pt.x;
      acc.sum_y = pt.y;
      acc.sum_z = pt.z;
      acc.sum_intensity = pt.intensity;
      accumulators_.push_back(acc);
    } else {
      const size_t idx = it->second;
      auto & v = voxels_[idx];
      auto & acc = accumulators_[idx];

      v.point_count = static_cast<uint16_t>(std::min(65535, v.point_count + 1));
      v.min_z = std::min(v.min_z, pt.z);
      v.max_z = std::max(v.max_z, pt.z);

      acc.sum_x += pt.x;
      acc.sum_y += pt.y;
      acc.sum_z += pt.z;
      acc.sum_intensity += pt.intensity;
    }
  }

  // Финальный расчет точных центроидов и заполнение продольных корзин Y
  const size_t total_voxels = voxels_.size();
  for (size_t i = 0; i < total_voxels; ++i) {
    auto & v = voxels_[i];
    const auto & acc = accumulators_[i];
    const double inv_cnt = 1.0 / static_cast<double>(v.point_count);
    v.x = static_cast<float>(acc.sum_x * inv_cnt);
    v.y = static_cast<float>(acc.sum_y * inv_cnt);
    v.z = static_cast<float>(acc.sum_z * inv_cnt);
    v.mean_intensity = static_cast<float>(acc.sum_intensity * inv_cnt);

    const float dist_y = -v.y;
    if (dist_y >= 0.0f && dist_y < config_.max_forward_range + 10.0f) {
      const int b = static_cast<int>(dist_y / bucket_size_y_);
      if (b >= 0 && b < num_y_buckets_) {
        y_bucket_indices_[b].push_back(i);
      }
    }
  }
}

void SparseVoxelGrid::query_frenet_slice(
  float center_x, float center_y, float yaw,
  float half_thickness, float max_lateral_dist,
  std::vector<const Voxel *> & out_voxels) const
{
  out_voxels.clear();

  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);

  const float forward_dist = -center_y;
  const float query_radius = half_thickness + max_lateral_dist * std::abs(sin_yaw) + 2.0f;

  const int b_min = std::max(0, static_cast<int>((forward_dist - query_radius) / bucket_size_y_));
  const int b_max = std::min(num_y_buckets_ - 1, static_cast<int>((forward_dist + query_radius) / bucket_size_y_));

  for (int b = b_min; b <= b_max; ++b) {
    for (size_t idx : y_bucket_indices_[b]) {
      const auto & v = voxels_[idx];
      const float dx = v.x - center_x;
      const float dy = v.y - center_y;

      // Проекция вокселя во фрейм Френе:
      // along-track: v_tangent
      // cross-track: u_normal
      const float v_tangent = dx * sin_yaw - dy * cos_yaw;
      if (std::abs(v_tangent) <= half_thickness) {
        const float u_normal = dx * cos_yaw + dy * sin_yaw;
        if (std::abs(u_normal) <= max_lateral_dist) {
          out_voxels.push_back(&v);
        }
      }
    }
  }
}

}  // namespace metro_voxel_tunnel_tracker
