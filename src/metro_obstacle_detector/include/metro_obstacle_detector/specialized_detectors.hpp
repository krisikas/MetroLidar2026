#pragma once

#include "metro_obstacle_detector/types.hpp"
#include "metro_obstacle_detector/clearance_envelope_sdf.hpp"
#include "metro_obstacle_detector/track_geometry_spline.hpp"
#include <vector>

namespace metro_obstacle_detector
{

struct DetectorConfig
{
  float cluster_dist_base{0.40f};      // Base clustering distance (m)
  float cluster_dist_slope{0.0025f};   // Range-adaptive expansion per meter
  uint32_t min_cluster_points{3};      // Minimum points for confirmed cluster
  float max_detection_range{150.0f};   // Max detection distance (m) set to 150m
  float min_detection_range{2.0f};     // Near zone cutoff (m)
  float voxel_size_base{0.15f};        // Base voxel size for spatial grid
};

class SpecializedDetectors
{
public:
  explicit SpecializedDetectors(
    const DetectorConfig & config = DetectorConfig{},
    const ClearanceEnvelopeConfig & env_config = ClearanceEnvelopeConfig{});

  void set_config(const DetectorConfig & config);
  const DetectorConfig & get_config() const { return config_; }

  void set_envelope_config(const ClearanceEnvelopeConfig & env_config)
  {
    envelope_sdf_.set_config(env_config);
  }
  const ClearanceEnvelopeConfig & get_envelope_config() const { return envelope_sdf_.get_config(); }

  /**
   * @brief Detect all obstacle candidates across multiple specialized layers
   *        and filter known static railway infrastructure.
   */
  void detect_obstacles(
    const std::vector<Point3D> & points,
    const std::vector<FrenetWaypoint> & waypoints,
    std::vector<Obstacle> & out_obstacles);

  /**
   * @brief Check whether point at Frenet coordinates (u, v) is part of static tunnel infrastructure:
   *        - Track bed / gate sill: v <= 0.12m
   *        - Contact rail: |u| in [1.05, 1.50]m at v in [0.05, 0.45]m
   *        - Station platform edge: |u| in [1.15, 1.85]m at v in [0.40, 1.25]m
   */
  bool is_infrastructure(float u_lat, float v_elev) const;

private:
  // Sub-detector 1: Bulk body clustering (DBSCAN / Euclidean with adaptive range voxelization)
  void detect_bulk_obstacles(
    const std::vector<Point3D> & points,
    const std::vector<FrenetWaypoint> & waypoints,
    std::vector<Obstacle> & candidates);

  // Sub-detector 2: Rail-mounted low bar (0.2m x 2m bar across rails, v in [0.12, 0.35]m, width >= 0.75m)
  void detect_rail_mounted_obstacles(
    const std::vector<Point3D> & points,
    const std::vector<FrenetWaypoint> & waypoints,
    std::vector<Obstacle> & candidates);

  // Sub-detector 3: Suspended thin filament (0.05m hanging cable from ceiling, v in [1.5, 2.75]m, vertical span >= 0.4m)
  void detect_suspended_filaments(
    const std::vector<Point3D> & points,
    const std::vector<FrenetWaypoint> & waypoints,
    std::vector<Obstacle> & candidates);

  // Classify obstacle threat based on exact signed distance to clearance boundary
  void classify_threat(
    Obstacle & obs,
    const std::vector<FrenetWaypoint> & waypoints);

  DetectorConfig config_;
  ClearanceEnvelopeSDF envelope_sdf_;
  TrackGeometrySpline track_spline_;
};

} // namespace metro_obstacle_detector
