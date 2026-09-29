#pragma once

#include "metro_obstacle_detector/types.hpp"
#include <vector>

namespace metro_obstacle_detector
{

struct SplineTrackerConfig
{
  float lookahead_distance{150.0f};    // Extended lookahead distance (m)
  float min_distance{2.0f};            // Near zone start in front of cab (m)
  float base_step{2.0f};               // Longitudinal discretization step (m)
  float min_curve_radius{160.0f};      // Minimum horizontal radius SP 120.13330 (m)
  float max_heading_angle{0.45f};      // Maximum tangent heading (rad)
  float nominal_tunnel_radius{2.15f};  // Standard circular tunnel radius (m)
};

class TrackGeometrySpline
{
public:
  explicit TrackGeometrySpline(const SplineTrackerConfig & config = SplineTrackerConfig{});

  void set_config(const SplineTrackerConfig & config);
  const SplineTrackerConfig & get_config() const { return config_; }

  // Fit smooth 3D track curve from dual-rail measurements and tunnel wall constraints
  void build_trajectory(
    const std::vector<Point3D> & points,
    const std::vector<DualRailMeasurement> & rail_measurements,
    std::vector<FrenetWaypoint> & out_waypoints);

  // Project any 3D point (x, y, z) onto the Frenet track ribbon: returns (s, u, v)
  // s = arc length, u = lateral offset (+right/-left), v = vertical height above rail crown
  bool project_to_frenet(
    float px, float py, float pz,
    const std::vector<FrenetWaypoint> & waypoints,
    float & out_s, float & out_u, float & out_v,
    size_t & out_closest_idx) const;

  void reset();

private:
  SplineTrackerConfig config_;
  std::vector<FrenetWaypoint> prev_waypoints_;
};

} // namespace metro_obstacle_detector
