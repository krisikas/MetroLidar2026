#pragma once

#include "metro_tunnel_tracker/types.hpp"
#include <vector>

namespace metro_tunnel_tracker
{

/**
 * @brief Конфигурация геометрического алгоритма трекинга пути
 */
struct TrackerConfig
{
  float lookahead_distance{150.0f};          ///< Горизонт трассировки пути вперед по ходу движения (м)
  float min_distance{2.5f};                  ///< Ближняя мертвая зона перед лидаром / сцепка поезда (м)
  float slice_step{2.5f};                    ///< Шаг продольного сечения тоннеля вдоль оси движения Y (м)
  float track_corridor_half_width{0.85f};    ///< Полуширина зоны поиска рельсов (|X| <= 0.85 м, колея 1520 мм)
  float clearance_corridor_half_width{1.75f};///< Полуширина габаритного коридора тоннеля по ГОСТ 9238 (м)
  float single_tunnel_radius{2.15f};         ///< Номинальный радиус круглого однопутного тоннеля метро (м)
  float min_curve_radius{180.0f};            ///< Нормативный минимальный радиус кривой пути метрополитена (м)
  float max_grade_slope{0.035f};             ///< Предельный уклон профиля пути по ПТЭ (35 тысячных / 3.5%)
  float default_rail_z{-1.35f};              ///< Проектная отметка головки рельса при старте в СК лидара (м)
  float gauge{1.520f};                       ///< Ширина русской колеи метрополитенов РФ (1520 мм)
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
