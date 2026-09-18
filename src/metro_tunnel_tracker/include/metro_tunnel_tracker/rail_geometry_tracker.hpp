#pragma once

#include "metro_tunnel_tracker/types.hpp"
#include <array>
#include <vector>

namespace metro_tunnel_tracker
{

/**
 * @brief Конфигурация геометрического алгоритма трекинга пути
 */
struct TrackerConfig
{
  float lookahead_distance{180.0f};          ///< Горизонт трассировки пути вперед по ходу движения (м, до 200 м)
  float min_distance{2.5f};                  ///< Ближняя мертвая зона перед лидаром / сцепка поезда (м)
  float slice_step{2.2f};                    ///< Базовый шаг продольного сечения для ближней зоны (м)
  float track_corridor_half_width{0.85f};    ///< Полуширина зоны поиска рельсов (|X| <= 0.85 м, колея 1520 мм)
  float clearance_corridor_half_width{1.75f};///< Полуширина габаритного коридора тоннеля по ГОСТ 9238 (м)
  float single_tunnel_radius{2.15f};         ///< Номинальный полугабарит однопутного тоннеля метро (м)
  float min_curve_radius{160.0f};            ///< Нормативный минимальный радиус кривой пути метрополитена (м)
  float max_grade_slope{0.035f};             ///< Предельный уклон профиля пути по ПТЭ (35 тысячных / 3.5%)
  float default_rail_z{-1.35f};              ///< Проектная отметка головки рельса при старте в СК лидара (м)
  float gauge{1.520f};                       ///< Ширина русской колеи метрополитенов РФ (1520 мм)

  // Range-Adaptive Slicing zone boundaries (distances from LiDAR, meters).
  // Zone 0 (near):   [min_distance .. zone1_start]  step = slice_step      (2.2 m)
  // Zone 1 (mid):    [zone1_start  .. zone2_start]  step = slice_step * 2  (4.4 m)
  // Zone 2 (far):    [zone2_start  .. lookahead]    step = slice_step * 4  (8.8 m)
  float zone1_start{35.5f};                 ///< Начало средней зоны (м)
  float zone2_start{79.5f};                 ///< Начало дальней зоны (м)

  /// Distance beyond which curvature updates are frozen (only heading is updated)
  float curvature_freeze_dist{75.0f};
};

class RailGeometryTracker
{
public:
  explicit RailGeometryTracker(const TrackerConfig & config = TrackerConfig());

  std::vector<TrackWaypoint> estimate_track_trajectory(const std::vector<Point3D> & points);

  const TrackerConfig & get_config() const { return config_; }
  void set_config(const TrackerConfig & config);

  /// Total number of adaptive slices (for external pre-allocation)
  int num_slices() const { return num_slices_; }

private:
  void init_buffers();

  TrackerConfig config_;
  int num_slices_{0};

  // Range-Adaptive Slicing structures

  /// Per-slice longitudinal step dy (meters): 2.2 / 4.4 / 8.8 depending on zone
  std::vector<float> slice_dy_;

  /// Per-slice Y center coordinate in LiDAR frame (negative, meters)
  std::vector<float> slice_y_centers_;

  /// O(1) Lookup Table: meter index -> slice index. Size = ceil(lookahead_distance) + 1.
  /// For a point at distance d meters, slice index = meter_to_slice_[int(d)].
  /// Value of -1 means the meter falls outside slicing range.
  std::vector<int> meter_to_slice_;

  // Pre-allocated scratch buffers to eliminate dynamic memory allocations in hot path (10 Hz)
  std::vector<std::vector<const Point3D *>> slice_ptrs_;
  std::vector<float> rail_head_zs_;
  std::vector<float> track_bed_zs_;
  std::vector<float> left_wall_xs_;
  std::vector<float> right_wall_xs_;
  std::vector<float> trough_xs_;
  std::vector<TrackWaypoint> trajectory_buf_;

  float estimate_slice_z(
    const std::vector<const Point3D *> & slice_ptrs,
    float prev_z,
    float x_pred,
    float z_pred,
    float dy,
    float & dz_dy);

  float estimate_slice_x(
    const std::vector<const Point3D *> & slice_ptrs,
    float prev_x,
    float z_rail,
    float dist_ahead,
    float dy,
    float & dx_dy,
    float & curvature,
    float & nominal_half_width,
    float & left_bound,
    float & right_bound,
    float & ceiling_z,
    float & confidence);
};

}  // namespace metro_tunnel_tracker
