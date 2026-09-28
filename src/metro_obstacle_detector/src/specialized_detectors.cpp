#include "metro_obstacle_detector/specialized_detectors.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace metro_obstacle_detector
{

SpecializedDetectors::SpecializedDetectors(
  const DetectorConfig & config,
  const ClearanceEnvelopeConfig & env_config)
: config_(config),
  envelope_sdf_(env_config)
{
}

void SpecializedDetectors::set_config(const DetectorConfig & config)
{
  config_ = config;
}

bool SpecializedDetectors::is_infrastructure(float u_lat, float v_elev) const
{
  // 1. Track bed / ballast / drainage trench
  if (v_elev <= 0.12f) {
    return true;
  }

  const float abs_u = std::abs(u_lat);

  // 2. Running rail heads and fasteners (|u| ~ 0.76m +- 0.12m, elevation <= 0.22m)
  if (abs_u <= 0.88f && v_elev <= 0.22f) {
    return true;
  }

  // 3. Contact rail casing (outside running rail, |u| in [1.12, 1.65]m at v in [0.05, 0.55]m)
  if (abs_u >= 1.12f && abs_u <= 1.65f && v_elev >= 0.05f && v_elev <= 0.55f) {
    return true;
  }

  // 4. Station platform edge (|u| in [1.32, 1.95]m at v in [0.35, 1.30]m)
  if (abs_u >= 1.32f && abs_u <= 1.95f && v_elev >= 0.35f && v_elev <= 1.30f) {
    return true;
  }

  return false;
}

void SpecializedDetectors::detect_obstacles(
  const std::vector<Point3D> & points,
  const std::vector<FrenetWaypoint> & waypoints,
  std::vector<Obstacle> & out_obstacles)
{
  out_obstacles.clear();
  if (points.empty() || waypoints.size() < 2) return;

  std::vector<Obstacle> candidates;
  candidates.reserve(64);

  // 1. Multi-scale Sub-detector 1: Bulk 3D Body Clustering
  detect_bulk_obstacles(points, waypoints, candidates);

  // 2. Multi-scale Sub-detector 2: Rail-Mounted Low Bar (0.2m x 2m bar across rails)
  detect_rail_mounted_obstacles(points, waypoints, candidates);

  // 3. Multi-scale Sub-detector 3: Suspended Thin Filament (0.05m wire hanging from ceiling)
  detect_suspended_filaments(points, waypoints, candidates);

  // 4. Merge duplicate or overlapping detections and assign threat levels
  for (auto & cand : candidates) {
    classify_threat(cand, waypoints);

    bool merged = false;
    for (auto & existing : out_obstacles) {
      const float dx = cand.center_x - existing.center_x;
      const float dy = cand.center_y - existing.center_y;
      const float dz = cand.center_z - existing.center_z;
      const float dist_sq = dx * dx + dy * dy + dz * dz;

      if (dist_sq <= 1.5f) { // Within 1.2m 3D radius
        existing.point_count += cand.point_count;
        existing.size_x = std::max(existing.size_x, cand.size_x);
        existing.size_y = std::max(existing.size_y, cand.size_y);
        existing.size_z = std::max(existing.size_z, cand.size_z);
        existing.gauge_penetration = std::min(existing.gauge_penetration, cand.gauge_penetration);

        if (existing.category == ObstacleCategory::BULK_BODY &&
            cand.category != ObstacleCategory::BULK_BODY) {
          existing.category = cand.category;
        }

        existing.confidence = std::max(existing.confidence, cand.confidence);
        if (static_cast<uint8_t>(cand.threat) > static_cast<uint8_t>(existing.threat)) {
          existing.threat = cand.threat;
        }
        merged = true;
        break;
      }
    }
    if (!merged) {
      out_obstacles.push_back(cand);
    }
  }

  // Sort by longitudinal distance (nearest first)
  std::sort(out_obstacles.begin(), out_obstacles.end(), [](const Obstacle & a, const Obstacle & b) {
    return a.distance < b.distance;
  });
}

void SpecializedDetectors::detect_bulk_obstacles(
  const std::vector<Point3D> & points,
  const std::vector<FrenetWaypoint> & waypoints,
  std::vector<Obstacle> & candidates)
{
  struct ProjectedPt {
    float x, y, z;
    float s, u, v;
    float sdf;
  };

  std::vector<ProjectedPt> filtered;
  filtered.reserve(points.size() / 16);

  const float traj_y_start = waypoints.front().y;
  const float traj_y_end = waypoints.back().y;

  for (const auto & p : points) {
    if (p.y > traj_y_start + 1.5f || p.y < traj_y_end - 1.5f) continue;

    float s = 0.0f, u = 0.0f, v = 0.0f;
    size_t wp_idx = 0;
    if (!track_spline_.project_to_frenet(p.x, p.y, p.z, waypoints, s, u, v, wp_idx)) continue;

    // Detection zone: min_detection_range to max_detection_range (150m)
    if (s < config_.min_detection_range || s > config_.max_detection_range) continue;

    // Infrastructure exclusion
    if (is_infrastructure(u, v)) continue;

    const float sdf = envelope_sdf_.compute_signed_distance(u, v);
    // ONLY keep points strictly inside clearance envelope
    if (sdf <= -0.02f) {
      filtered.push_back(ProjectedPt{p.x, p.y, p.z, s, u, v, sdf});
    }
  }

  if (filtered.empty()) return;

  // Spatial clustering with 0.50m grid
  struct VoxelNode {
    double sum_x{0.0}, sum_y{0.0}, sum_z{0.0};
    double sum_s{0.0}, sum_u{0.0}, sum_v{0.0};
    float min_sdf{1e6f};
    uint32_t count{0};
  };

  auto make_key = [](int32_t kx, int32_t ky, int32_t kz) -> uint64_t {
    const uint64_t ux = static_cast<uint64_t>(kx + 32768) & 0x1FFFFF;
    const uint64_t uy = static_cast<uint64_t>(ky + 32768) & 0x1FFFFF;
    const uint64_t uz = static_cast<uint64_t>(kz + 32768) & 0x1FFFFF;
    return (ux << 42) | (uy << 21) | uz;
  };

  const float voxel_res = 0.20f;
  std::unordered_map<uint64_t, VoxelNode> voxel_map;
  voxel_map.reserve(filtered.size());

  for (const auto & fp : filtered) {
    const int32_t kx = static_cast<int32_t>(std::floor(fp.x / voxel_res));
    const int32_t ky = static_cast<int32_t>(std::floor(fp.y / voxel_res));
    const int32_t kz = static_cast<int32_t>(std::floor(fp.z / voxel_res));

    const uint64_t key = make_key(kx, ky, kz);
    auto & node = voxel_map[key];
    node.sum_x += fp.x;
    node.sum_y += fp.y;
    node.sum_z += fp.z;
    node.sum_s += fp.s;
    node.sum_u += fp.u;
    node.sum_v += fp.v;
    node.min_sdf = std::min(node.min_sdf, fp.sdf);
    node.count += 1;
  }

  struct VoxelPt {
    float x, y, z;
    float s, u, v;
    float sdf;
    uint32_t raw_pts;
  };

  std::vector<VoxelPt> voxels;
  voxels.reserve(voxel_map.size());
  for (const auto & kv : voxel_map) {
    const auto & n = kv.second;
    const double inv = 1.0 / static_cast<double>(n.count);
    voxels.push_back(VoxelPt{
      static_cast<float>(n.sum_x * inv),
      static_cast<float>(n.sum_y * inv),
      static_cast<float>(n.sum_z * inv),
      static_cast<float>(n.sum_s * inv),
      static_cast<float>(n.sum_u * inv),
      static_cast<float>(n.sum_v * inv),
      n.min_sdf,
      n.count
    });
  }

  const float bucket_size = 0.55f;
  std::unordered_map<uint64_t, std::vector<size_t>> spatial_grid;
  spatial_grid.reserve(voxels.size());

  for (size_t i = 0; i < voxels.size(); ++i) {
    const int32_t bx = static_cast<int32_t>(std::floor(voxels[i].x / bucket_size));
    const int32_t by = static_cast<int32_t>(std::floor(voxels[i].y / bucket_size));
    const int32_t bz = static_cast<int32_t>(std::floor(voxels[i].z / bucket_size));
    spatial_grid[make_key(bx, by, bz)].push_back(i);
  }

  const size_t n_vox = voxels.size();
  std::vector<bool> visited(n_vox, false);
  const float eps = 0.60f;
  const float eps_sq = eps * eps;

  for (size_t i = 0; i < n_vox; ++i) {
    if (visited[i]) continue;

    std::vector<size_t> cluster_indices;
    std::queue<size_t> q;
    visited[i] = true;
    q.push(i);

    while (!q.empty()) {
      const size_t curr = q.front();
      q.pop();
      cluster_indices.push_back(curr);

      const auto & curr_pt = voxels[curr];
      const int32_t cbx = static_cast<int32_t>(std::floor(curr_pt.x / bucket_size));
      const int32_t cby = static_cast<int32_t>(std::floor(curr_pt.y / bucket_size));
      const int32_t cbz = static_cast<int32_t>(std::floor(curr_pt.z / bucket_size));

      for (int32_t dx = -1; dx <= 1; ++dx) {
        for (int32_t dy = -1; dy <= 1; ++dy) {
          for (int32_t dz = -1; dz <= 1; ++dz) {
            const auto it = spatial_grid.find(make_key(cbx + dx, cby + dy, cbz + dz));
            if (it == spatial_grid.end()) continue;

            for (size_t neighbor_idx : it->second) {
              if (!visited[neighbor_idx]) {
                const auto & nb = voxels[neighbor_idx];
                const float ddx = curr_pt.x - nb.x;
                const float ddy = curr_pt.y - nb.y;
                const float ddz = curr_pt.z - nb.z;
                if (ddx * ddx + ddy * ddy + ddz * ddz <= eps_sq) {
                  visited[neighbor_idx] = true;
                  q.push(neighbor_idx);
                }
              }
            }
          }
        }
      }
    }

    uint32_t total_raw_points = 0;
    for (size_t idx : cluster_indices) {
      total_raw_points += voxels[idx].raw_pts;
    }

    const float dist_ahead = voxels[cluster_indices.front()].s;
    const uint32_t min_pts = (dist_ahead >= 70.0f) ? 4 : ((dist_ahead >= 35.0f) ? 5 : 6);

    if (total_raw_points >= min_pts) {
      float min_x = 1e6f, max_x = -1e6f;
      float min_y = 1e6f, max_y = -1e6f;
      float min_z = 1e6f, max_z = -1e6f;
      double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
      float min_sdf = 1e6f;

      for (size_t idx : cluster_indices) {
        const auto & vp = voxels[idx];
        min_x = std::min(min_x, vp.x);
        max_x = std::max(max_x, vp.x);
        min_y = std::min(min_y, vp.y);
        max_y = std::max(max_y, vp.y);
        min_z = std::min(min_z, vp.z);
        max_z = std::max(max_z, vp.z);
        sum_x += vp.x * vp.raw_pts;
        sum_y += vp.y * vp.raw_pts;
        sum_z += vp.z * vp.raw_pts;
        min_sdf = std::min(min_sdf, vp.sdf);
      }

      const float span_x = max_x - min_x;
      const float span_z = max_z - min_z;

      // Reject oversized wall structures
      if (span_x > 2.80f || span_z > 2.80f) continue;

      // Reject distant flat grazing noise on trackbed
      if (dist_ahead >= 50.0f && span_z < 0.25f && (min_z - waypoints.front().z) < 0.50f) continue;
      if (dist_ahead >= 85.0f && span_z < 0.28f && (min_z - waypoints.front().z) < 0.55f) continue;

      // Reject overhead ceiling traverse beams (cable/cantilever gantries)
      if (span_x > 1.20f && span_z <= 0.18f && (min_z - waypoints.front().z) >= 1.80f) continue;

      const double inv_total = 1.0 / static_cast<double>(total_raw_points);
      Obstacle obs;
      obs.center_x = static_cast<float>(sum_x * inv_total);
      obs.center_y = static_cast<float>(sum_y * inv_total);
      obs.center_z = static_cast<float>(sum_z * inv_total);
      obs.size_x = std::max(0.20f, span_x);
      obs.size_y = std::max(0.20f, max_y - min_y);
      obs.size_z = std::max(0.15f, span_z);
      obs.distance = -obs.center_y;
      obs.point_count = total_raw_points;
      obs.gauge_penetration = min_sdf;
      obs.category = ObstacleCategory::BULK_BODY;
      obs.confidence = std::min(1.0f, 0.50f + 0.05f * static_cast<float>(total_raw_points));

      candidates.push_back(obs);
    }
  }
}

void SpecializedDetectors::detect_rail_mounted_obstacles(
  const std::vector<Point3D> & points,
  const std::vector<FrenetWaypoint> & waypoints,
  std::vector<Obstacle> & candidates)
{
  struct RailBarPt {
    const Point3D * pt;
    float s, u, v;
  };

  std::vector<RailBarPt> low_bar_pts;
  low_bar_pts.reserve(512);

  for (const auto & p : points) {
    if (p.y > waypoints.front().y + 1.0f || p.y < waypoints.back().y - 1.0f) continue;

    float s = 0.0f, u = 0.0f, v = 0.0f;
    size_t wp_idx = 0;
    if (!track_spline_.project_to_frenet(p.x, p.y, p.z, waypoints, s, u, v, wp_idx)) continue;

    if (s < config_.min_detection_range || s > config_.max_detection_range) continue;

    // Low bar on rails: elevated above rail heads v in [0.25, 0.48]m, across track |u| <= 0.90m
    if (v >= 0.25f && v <= 0.48f && std::abs(u) <= 0.90f) {
      low_bar_pts.push_back(RailBarPt{&p, s, u, v});
    }
  }

  if (low_bar_pts.size() < 8) return;

  // Group by longitudinal s
  std::sort(low_bar_pts.begin(), low_bar_pts.end(), [](const RailBarPt & a, const RailBarPt & b) {
    return a.s < b.s;
  });

  std::vector<std::vector<RailBarPt>> groups;
  groups.push_back({low_bar_pts[0]});

  for (size_t i = 1; i < low_bar_pts.size(); ++i) {
    if (std::abs(low_bar_pts[i].s - groups.back().back().s) <= 0.40f) {
      groups.back().push_back(low_bar_pts[i]);
    } else {
      groups.push_back({low_bar_pts[i]});
    }
  }

  for (const auto & grp : groups) {
    if (grp.size() < 8) continue;

    float min_x = 1e6f, max_x = -1e6f;
    float min_y = 1e6f, max_y = -1e6f;
    float min_z = 1e6f, max_z = -1e6f;
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

    for (const auto & rbp : grp) {
      min_x = std::min(min_x, rbp.pt->x);
      max_x = std::max(max_x, rbp.pt->x);
      min_y = std::min(min_y, rbp.pt->y);
      max_y = std::max(max_y, rbp.pt->y);
      min_z = std::min(min_z, rbp.pt->z);
      max_z = std::max(max_z, rbp.pt->z);
      sum_x += rbp.pt->x;
      sum_y += rbp.pt->y;
      sum_z += rbp.pt->z;
    }

    const float span_x = max_x - min_x;
    const float span_z = max_z - min_z;

    // Requirement: bar across rails: lateral span >= 0.70m, vertical span <= 0.35m
    if (span_x >= 0.70f && span_z <= 0.35f) {
      const double inv_n = 1.0 / static_cast<double>(grp.size());
      Obstacle obs;
      obs.center_x = static_cast<float>(sum_x * inv_n);
      obs.center_y = static_cast<float>(sum_y * inv_n);
      obs.center_z = static_cast<float>(sum_z * inv_n);
      obs.size_x = span_x;
      obs.size_y = std::max(0.15f, max_y - min_y);
      obs.size_z = std::max(0.10f, span_z);
      obs.distance = -obs.center_y;
      obs.point_count = static_cast<uint32_t>(grp.size());
      obs.category = ObstacleCategory::RAIL_SURFACE;
      obs.confidence = 0.95f;
      obs.gauge_penetration = -0.20f;
      candidates.push_back(obs);
    }
  }
}

void SpecializedDetectors::detect_suspended_filaments(
  const std::vector<Point3D> & points,
  const std::vector<FrenetWaypoint> & waypoints,
  std::vector<Obstacle> & candidates)
{
  struct FilamentPt {
    const Point3D * pt;
    float s, u, v;
  };

  std::vector<FilamentPt> filament_pts;
  filament_pts.reserve(256);

  for (const auto & p : points) {
    if (p.y > waypoints.front().y + 1.0f || p.y < waypoints.back().y - 1.0f) continue;

    float s = 0.0f, u = 0.0f, v = 0.0f;
    size_t wp_idx = 0;
    if (!track_spline_.project_to_frenet(p.x, p.y, p.z, waypoints, s, u, v, wp_idx)) continue;

    if (s < 3.5f || s > config_.max_detection_range) continue;

    // Suspended filament: narrow lateral (|u| <= 0.50m), high vertical (v in [1.50, 2.80]m)
    if (std::abs(u) <= 0.50f && v >= 1.50f && v <= 2.80f) {
      filament_pts.push_back(FilamentPt{&p, s, u, v});
    }
  }

  if (filament_pts.size() < 4) return;

  std::sort(filament_pts.begin(), filament_pts.end(), [](const FilamentPt & a, const FilamentPt & b) {
    return a.s < b.s;
  });

  std::vector<std::vector<FilamentPt>> groups;
  groups.push_back({filament_pts[0]});

  for (size_t i = 1; i < filament_pts.size(); ++i) {
    if (std::abs(filament_pts[i].s - groups.back().back().s) <= 0.50f) {
      groups.back().push_back(filament_pts[i]);
    } else {
      groups.push_back({filament_pts[i]});
    }
  }

  for (const auto & grp : groups) {
    if (grp.size() < 4) continue;

    float min_x = 1e6f, max_x = -1e6f;
    float min_y = 1e6f, max_y = -1e6f;
    float min_z = 1e6f, max_z = -1e6f;
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

    for (const auto & fp : grp) {
      min_x = std::min(min_x, fp.pt->x);
      max_x = std::max(max_x, fp.pt->x);
      min_y = std::min(min_y, fp.pt->y);
      max_y = std::max(max_y, fp.pt->y);
      min_z = std::min(min_z, fp.pt->z);
      max_z = std::max(max_z, fp.pt->z);
      sum_x += fp.pt->x;
      sum_y += fp.pt->y;
      sum_z += fp.pt->z;
    }

    const float vert_span = max_z - min_z;
    const float lateral_width = max_x - min_x;

    // Requirement: suspended cable: vertical span >= 0.35m, lateral width <= 0.35m
    if (vert_span >= 0.35f && lateral_width <= 0.35f) {
      const double inv_n = 1.0 / static_cast<double>(grp.size());
      Obstacle obs;
      obs.center_x = static_cast<float>(sum_x * inv_n);
      obs.center_y = static_cast<float>(sum_y * inv_n);
      obs.center_z = static_cast<float>(sum_z * inv_n);
      obs.size_x = std::max(0.05f, lateral_width);
      obs.size_y = std::max(0.10f, max_y - min_y);
      obs.size_z = std::max(0.35f, vert_span);
      obs.distance = -obs.center_y;
      obs.point_count = static_cast<uint32_t>(grp.size());
      obs.category = ObstacleCategory::SUSPENDED_CABLE;
      obs.confidence = 0.90f;
      obs.gauge_penetration = -0.30f;
      candidates.push_back(obs);
    }
  }
}

void SpecializedDetectors::classify_threat(
  Obstacle & obs,
  const std::vector<FrenetWaypoint> & waypoints)
{
  float s = 0.0f, u = 0.0f, v = 0.0f;
  size_t wp_idx = 0;
  if (!track_spline_.project_to_frenet(obs.center_x, obs.center_y, obs.center_z, waypoints, s, u, v, wp_idx)) {
    obs.threat = ThreatLevel::NONE;
    return;
  }

  const float sdf = envelope_sdf_.compute_signed_distance(u, v);
  obs.gauge_penetration = sdf;

  if (sdf <= -0.05f) {
    obs.threat = (obs.distance < 80.0f) ? ThreatLevel::CRITICAL : ThreatLevel::WARNING;
  } else if (sdf <= 0.0f) {
    obs.threat = ThreatLevel::WARNING;
  } else {
    obs.threat = ThreatLevel::CAUTION;
  }
}

} // namespace metro_obstacle_detector
