#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>
#include <array>
#include <cmath>

namespace metro_obstacle_detector
{

struct Point3D
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
  uint16_t ring{0};
  double timestamp{0.0};
};

struct VoxelCentroid
{
  int32_t ix{0};
  int32_t iy{0};
  int32_t iz{0};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
  uint32_t count{0};
};

struct DualRailMeasurement
{
  float left_rail_x{-0.760f};
  float left_rail_z{-1.075f};
  float right_rail_x{0.760f};
  float right_rail_z{-1.075f};
  float crown_z{-1.075f};       // Level of Rail Head (УГР)
  float bed_z{-1.350f};         // Track bed level
  float track_center_x{0.0f};
  float roll_angle{0.0f};       // Cant / superelevation
  float confidence{0.0f};       // [0.0 - 1.0]
  bool left_detected{false};
  bool right_detected{false};
};

struct FrenetWaypoint
{
  float s{0.0f};                // Arc length distance along track (m)
  float x{0.0f};                // 3D Cartesian position
  float y{0.0f};
  float z{0.0f};                // Rail crown elevation
  float yaw{0.0f};              // Tangent heading (rad)
  float pitch{0.0f};            // Grade angle (rad)
  float roll{0.0f};             // Superelevation (rad)
  float curvature{0.0f};        // Curvature kappa (1/m)
  float left_wall{2.15f};       // Distance to left wall (m)
  float right_wall{2.15f};      // Distance to right wall (m)
  float confidence{1.0f};
};

enum class ThreatLevel : uint8_t
{
  NONE = 0,
  CAUTION = 1,      // Near gauge / buffer zone
  WARNING = 2,      // Inside gauge, normal deceleration distance
  CRITICAL = 3      // Emergency brake required immediately
};

enum class ObstacleCategory : uint8_t
{
  UNKNOWN = 0,
  BULK_BODY = 1,        // Standard 3D volumetric obstacles (2x2m, 0.3x0.3m)
  RAIL_SURFACE = 2,     // Object lying directly on rails (2x0.2m)
  SUSPENDED_CABLE = 3   // Vertical hanging line/wire (0.05m width)
};

struct Obstacle
{
  uint32_t id{0};
  float distance{0.0f};         // Distance ahead of train (m)
  float center_x{0.0f};
  float center_y{0.0f};
  float center_z{0.0f};
  float size_x{0.0f};
  float size_y{0.0f};
  float size_z{0.0f};
  float velocity_x{0.0f};
  float velocity_y{0.0f};
  float velocity_z{0.0f};
  float confidence{0.0f};       // [0.0 - 1.0]
  float gauge_penetration{0.0f};// Signed distance (negative = inside, positive = outside)
  ThreatLevel threat{ThreatLevel::NONE};
  ObstacleCategory category{ObstacleCategory::UNKNOWN};
  uint32_t point_count{0};
  uint32_t tracking_frames{1};
  bool confirmed{false};
};

enum class SafetyStatus : uint8_t
{
  CLEAR = 0,
  NEAR_GAUGE_ADVISORY = 1,
  TRACK_WARNING = 2,
  EMERGENCY_BRAKE = 3
};

struct SafetyDecision
{
  SafetyStatus status{SafetyStatus::CLEAR};
  float train_speed{0.0f};              // m/s
  float min_distance_to_obstacle{1e6f}; // meters
  float time_to_collision{-1.0f};       // seconds
  float braking_distance{0.0f};         // meters
  bool emergency_brake_required{false};
  std::vector<Obstacle> active_obstacles;
};

struct SystemTelemetry
{
  float processing_time_ms{0.0f};
  float fps{0.0f};
  uint32_t input_points{0};
  uint32_t valid_points{0};
  uint32_t voxel_count{0};
  uint32_t candidate_clusters{0};
  uint32_t confirmed_obstacles{0};
  uint32_t active_tracks{0};
  bool gpu_accelerated{false};
};

} // namespace metro_obstacle_detector
