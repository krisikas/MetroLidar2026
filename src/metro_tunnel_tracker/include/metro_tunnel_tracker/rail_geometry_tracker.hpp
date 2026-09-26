#pragma once

#include "metro_tunnel_tracker/types.hpp"
#include <array>
#include <vector>

namespace metro_tunnel_tracker
{

/**
 * @brief Полная конфигурация геометрического алгоритма трекинга пути и тоннеля
 */
struct TrackerConfig
{
  // 1. Продольная дискретизация и горизонт трассировки пути
  float lookahead_distance{180.0f};          ///< Горизонт трассировки пути вперед по ходу движения (м, до 200 м)
  float min_distance{2.5f};                  ///< Ближняя мертвая зона перед лидаром / сцепка поезда (м)
  float slice_step{2.2f};                    ///< Базовый шаг продольного сечения для ближней зоны (м)
  float zone1_start{35.5f};                  ///< Дистанция перехода в среднюю зону (шаг x2) (м)
  float zone2_start{79.5f};                  ///< Дистанция перехода в дальнюю зону (шаг x4) (м)
  float curvature_freeze_dist{75.0f};        ///< Дистанция заморозки обновления кривизны (м)

  // 2. Железнодорожные нормативы и геометрия пути
  float track_corridor_half_width{0.85f};    ///< Полуширина зоны поиска рельсов (|u| <= 0.85 м, колея 1520 мм)
  float clearance_corridor_half_width{1.75f};///< Полуширина габаритного коридора тоннеля по ГОСТ 9238 (м)
  float single_tunnel_radius{2.15f};         ///< Номинальный полугабарит однопутного тоннеля метро (м)
  float min_curve_radius{160.0f};            ///< Нормативный минимальный радиус кривой пути метрополитена (м)
  float max_grade_slope{0.035f};             ///< Предельный уклон профиля пути по ПТЭ (3.5%)
  float default_rail_z{-1.35f};              ///< Проектная отметка головки рельса при старте в СК лидара (м)
  float gauge{1.520f};                       ///< Ширина русской колеи метрополитенов РФ (1520 мм)
  float max_heading_slope{0.45f};            ///< Максимальный тангенс угла рыскания пути

  // 3. Параметры поиска рельсов и УГР
  float rail_search_tolerance{0.16f};        ///< Допуск поиска головки рельса по ширине (|u - rail| <= tol) (м)
  float rail_z_min_offset{-0.35f};           ///< Нижняя граница поиска ходового полотна относительно z_pred (м)
  float rail_z_max_offset{0.30f};            ///< Верхняя граница поиска ходового полотна относительно z_pred (м)
  float rail_head_quantile{0.85f};           ///< Квантиль высоты для головок рельсов (0.0 - 1.0)
  float track_bed_quantile{0.85f};           ///< Квантиль высоты для балластного полотна (0.0 - 1.0)
  float rail_height_over_bed{0.16f};         ///< Конструктивное превышение головки рельса над полотном (м)
  int min_rail_points{3};                    ///< Минимальное число точек для достоверного захвата рельсов

  // 4. Параметры поиска стен и габаритов тоннеля
  float wall_search_min_dist{1.25f};         ///< Минимальное расстояние от оси пути до стены тоннеля (м)
  float wall_search_max_dist{4.20f};         ///< Максимальное расстояние поиска стены тоннеля (м)
  float wall_z_min{0.35f};                   ///< Нижняя граница поиска стен над УГР (м)
  float wall_z_max{3.80f};                   ///< Верхняя граница поиска стен над УГР (м)
  float left_wall_quantile{0.90f};           ///< Квантиль внутренней поверхности левой стены (0.0 - 1.0)
  float right_wall_quantile{0.10f};          ///< Квантиль внутренней поверхности правой стены (0.0 - 1.0)
  int min_wall_points_near{4};               ///< Мин. точек стены в ближней зоне (до wall_near_threshold)
  int min_wall_points_far{3};                ///< Мин. точек стены в дальней зоне
  float wall_near_threshold{45.0f};          ///< Граница ближней/дальней зоны плотности стен (м)
  float symmetric_tunnel_min_width{3.0f};    ///< Мин. ширина симметричного однопутного тоннеля (м)
  float symmetric_tunnel_max_width{5.4f};    ///< Макс. ширина симметричного однопутного тоннеля (м)
  float symmetric_tunnel_wall_tolerance{1.2f};///< Допустимая асимметрия стен для центрирования (м)
  float wall_tracking_error_tolerance{0.70f};///< Допуск удержания стены в асимметричных участках (м)
  float single_wall_tolerance{0.85f};        ///< Допуск удержания единственной стены на повороте (м)

  // 5. Фильтрация сырых точек
  float max_lateral_offset{12.0f};           ///< Максимальное боковое отклонение точек от оси лидара (|x| <= max) (м)
};

class RailGeometryTracker
{
public:
  explicit RailGeometryTracker(const TrackerConfig & config = TrackerConfig());

  /**
   * @brief Оценка 3D-траектории пути с использованием адаптивного среза по ходу кривизны (Frenet-Serret slicing)
   * @param points Входное облако 3D-точек в СК лидара
   * @return Вектор опорных вейпоинтов пути с углами yaw, pitch и границами тоннеля
   */
  std::vector<TrackWaypoint> estimate_track_trajectory(const std::vector<Point3D> & points);

  const TrackerConfig & get_config() const { return config_; }
  void set_config(const TrackerConfig & config);

  /// Общее число адаптивных срезов
  int num_slices() const { return num_slices_; }

private:
  void init_buffers();

  TrackerConfig config_;
  int num_slices_{0};
  float bucket_step_{2.0f};
  int num_buckets_{0};

  /// Шаги срезов ds вдоль криволинейной траектории
  std::vector<float> slice_ds_;

  /// Предвыделенные пространственные корзины по Y для быстрого O(1) поиска точек
  std::vector<std::vector<const Point3D *>> y_buckets_;

  // Буферы для устранения аллокаций памяти в цикле 10 Гц
  std::vector<float> rail_head_zs_;
  std::vector<float> track_bed_zs_;
  std::vector<float> left_wall_us_;
  std::vector<float> right_wall_us_;
  std::vector<TrackWaypoint> trajectory_buf_;
};

}  // namespace metro_tunnel_tracker
