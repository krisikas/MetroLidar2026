#pragma once

#include "metro_obstacle_detector/types.hpp"
#include <vector>

namespace metro_obstacle_detector
{

struct DualRailConfig
{
  float gauge{1.520f};                 // Russian standard gauge G = 1520 mm (m)
  float gauge_tolerance{0.080f};       // Gauge tolerance +-80 mm (m) [1.440m - 1.600m]
  float rail_search_half_width{0.14f}; // Lateral window around rail head (m)
  float nominal_tor_height{1.075f};    // Nominal LiDAR height above top of rail (m)
  float max_grade_slope{0.035f};       // Maximum PTE longitudinal slope (35 permille)
  float min_rail_points{3};            // Minimum points to confirm rail crown
  float bed_to_rail_height{0.18f};     // Height difference between rail crown and bed (m)
  float default_rail_z{-1.075f};       // Default Z in LiDAR frame (m)
  float max_cant_angle{0.08f};         // Maximum track cant / roll angle (rad)
};

class DualRailTracker
{
public:
  explicit DualRailTracker(const DualRailConfig & config = DualRailConfig{});

  void set_config(const DualRailConfig & config);
  const DualRailConfig & get_config() const { return config_; }

  /**
   * @brief Estimate rail crowns, track bed, gauge, cant, and track centerline in a longitudinal slice
   *        using dual-rail correlation tracking with G = 1520 mm (+-80 mm).
   */
  DualRailMeasurement estimate_slice(
    const std::vector<Point3D> & slice_points,
    float x_prior,
    float y_center,
    float z_prior,
    float yaw_prior);

  /**
   * @brief Smooth dynamic longitudinal rail elevation (Z_rail) profile across consecutive waypoints
   *        enforcing railway physical grade limits (<= 3.5%).
   */
  void smooth_profile(std::vector<FrenetWaypoint> & waypoints);

  void reset();

private:
  DualRailConfig config_;
  float last_valid_crown_z_{-1.075f};
  float dynamic_grade_slope_{0.0f};
};

} // namespace metro_obstacle_detector
