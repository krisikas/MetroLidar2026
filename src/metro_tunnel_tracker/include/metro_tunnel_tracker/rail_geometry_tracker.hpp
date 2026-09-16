#pragma once

#include "metro_tunnel_tracker/types.hpp"
#include <vector>

namespace metro_tunnel_tracker
{

struct TrackerConfig
{
  float lookahead_distance{150.0f};
  float min_distance{2.5f};
  float slice_step{2.5f};
  float track_corridor_half_width{0.85f};
  float clearance_corridor_half_width{1.75f};
  float single_tunnel_radius{2.15f};
  float min_curve_radius{180.0f};
  float max_grade_slope{0.035f};
  float default_rail_z{-1.35f};
  float gauge{1.520f};
};


class RailGeometryTracker
{
public:
  explicit RailGeometryTracker(const TrackerConfig & config = TrackerConfig());

  std::vector<TrackWaypoint> estimate_track_trajectory(const std::vector<Point3D> & points);

  const TrackerConfig & get_config() const { return config_; }
  void set_config(const TrackerConfig & config) { config_ = config; }

private:
  TrackerConfig config_;

  float estimate_slice_z(
    const std::vector<Point3D> & slice_points,
    float x_pred,
    float z_pred,
    float & dz_dy);

  float estimate_slice_x(
    const std::vector<Point3D> & slice_points,
    float prev_x,
    float z_rail,
    float dist_ahead,
    float & dx_dy,
    float & curvature,
    float & left_bound,
    float & right_bound);
};

}  // namespace metro_tunnel_tracker
