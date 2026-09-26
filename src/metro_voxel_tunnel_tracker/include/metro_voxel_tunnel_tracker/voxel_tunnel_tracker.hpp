#pragma once

#include "metro_voxel_tunnel_tracker/types.hpp"
#include "metro_voxel_tunnel_tracker/voxel_grid.hpp"
#include <vector>

namespace metro_voxel_tunnel_tracker
{

/**
 * @brief Конфигурация воксельного трекера геометрии пути и тоннеля
 */
struct VoxelTrackerConfig
{
  // 1. Продольная дискретизация пути
  float lookahead_distance{180.0f};          ///< Горизонт трассировки пути вперед (м, до 200 м)
  float min_distance{2.5f};                  ///< Ближняя мертвая зона перед лидаром / сцепка (м)
  float slice_step{2.2f};                    ///< Базовый шаг продольного среза для ближней зоны (м)
  float zone1_start{35.5f};                  ///< Дистанция перехода в среднюю зону (шаг x2) (м)
  float zone2_start{79.5f};                  ///< Дистанция перехода в дальнюю зону (шаг x4) (м)
  float curvature_freeze_dist{75.0f};        ///< Дистанция заморозки обновления кривизны (м)

  // 2. Нормативы железнодорожной колеи и тоннеля
  float track_corridor_half_width{0.85f};    ///< Полуширина зоны рельсов (|u| <= 0.85 м, колея 1520 мм)
  float clearance_corridor_half_width{1.75f};///< Полуширина габаритного коридора тоннеля по ГОСТ 9238 (м)
  float single_tunnel_radius{2.15f};         ///< Номинальный полугабарит тоннеля (м)
  float min_curve_radius{160.0f};            ///< Нормативный минимальный радиус кривой пути метрополитена (м)
  float max_grade_slope{0.035f};             ///< Предельный продольный уклон пути по ПТЭ (3.5%)
  float default_rail_z{-1.35f};              ///< Проектная отметка головки рельса при старте в СК лидара (м)
  float gauge{1.520f};                       ///< Ширина русской колеи (1520 мм)
  float max_heading_slope{0.45f};            ///< Максимальный тангенс угла рыскания пути

  // 3. Параметры поиска рельсов и УГР по вокселям
  float rail_search_tolerance{0.16f};        ///< Допуск поиска головки рельса по ширине (|u - rail| <= tol) (м)
  float rail_z_min_offset{-0.35f};           ///< Нижняя граница поиска ходового полотна относительно z_pred (м)
  float rail_z_max_offset{0.30f};            ///< Верхняя граница поиска ходового полотна относительно z_pred (м)
  float rail_head_quantile{0.85f};           ///< Квантиль высоты для головок рельсов (0.0 - 1.0)
  float track_bed_quantile{0.85f};           ///< Квантиль высоты для балластного полотна (0.0 - 1.0)
  float rail_height_over_bed{0.16f};         ///< Конструктивное превышение головки рельса над лотком/полотном (м)
  int min_rail_voxels{2};                    ///< Мин. число вокселей рельса для достоверного УГР

  // 4. Форма-инвариантные параметры поиска стен и границ свободного коридора
  float wall_search_min_dist{1.25f};         ///< Мин. расстояние от оси пути до стены (м)
  float wall_search_max_dist{4.20f};         ///< Макс. расстояние поиска стены (м)
  float wall_z_min{0.35f};                   ///< Нижняя граница поиска стен над УГР (м)
  float wall_z_max{3.80f};                   ///< Верхняя граница поиска стен над УГР (м)
  float left_wall_quantile{0.90f};           ///< Квантиль внутренней кромки левой стены
  float right_wall_quantile{0.10f};          ///< Квантиль внутренней кромки правой стены
  int min_wall_voxels_near{3};               ///< Мин. вокселей стены в ближней зоне
  int min_wall_voxels_far{2};                ///< Мин. вокселей стены в дальней зоне
  float wall_near_threshold{45.0f};          ///< Граница ближней/дальней зоны плотности стен (м)
  float symmetric_tunnel_min_width{3.0f};    ///< Мин. ширина симметричного однопутного тоннеля (м)
  float symmetric_tunnel_max_width{5.4f};    ///< Макс. ширина симметричного однопутного тоннеля (м)
  float symmetric_tunnel_wall_tolerance{1.2f};///< Допустимая асимметрия стен для центрирования (м)
  float wall_tracking_error_tolerance{0.70f};///< Допуск удержания стены при асимметрии (м)
  float single_wall_tolerance{0.85f};        ///< Допуск удержания единственной стены на кривой (м)

  // 5. Параметры контроля габарита подвижного состава (ГОСТ 9238)
  float rail_head_clearance{0.18f};          ///< Зазор над УГР (исключение рельсов и стыков) (м)
  float undercarriage_half_width{1.15f};     ///< Полуширина подвагонной зоны между контактным рельсом (м)
  float contact_rail_height{0.60f};          ///< Верхняя граница зоны контактного рельса над УГР (м)
  float platform_clearance_half_width{1.33f};///< Полуширина на уровне пассажирской платформы (м)
  float platform_height{1.25f};              ///< Высота станционной платформы над УГР (м)
  float waist_half_width{1.37f};             ///< Полуширина кузова выше платформы (м)
  float carriage_wall_height{2.60f};         ///< Высота вертикальной стенки кузова над УГР (м)
  float roof_half_width{0.85f};              ///< Верхняя полуширина крышевого ската (м)
  float carriage_height{3.60f};              ///< Полная габаритная высота вагона над УГР (м)
  int min_cluster_voxels{3};                 ///< Мин. вокселей для регистрации препятствия
  float cluster_distance_thresh{0.45f};      ///< Радиус объединения вокселей в кластер препятствия (м)

  // 6. Сетка вокселей
  VoxelGridConfig voxel_config;
};

/**
 * @brief Алгоритмический процессор воксельного трекинга пути и контроля свободности габарита
 */
class VoxelTunnelTracker
{
public:
  explicit VoxelTunnelTracker(const VoxelTrackerConfig & config = VoxelTrackerConfig());

  void set_config(const VoxelTrackerConfig & config);
  const VoxelTrackerConfig & get_config() const { return config_; }

  /**
   * @brief Полный цикл обработки: вокселизация, трекинг траектории и поиск препятствий
   * @param points Входные сырые 3D-точки лидара
   * @param out_trajectory Выходная 3D-траектория пути
   * @param out_obstacles Выходной список кластеров препятствий
   */
  void process(
    const std::vector<Point3D> & points,
    std::vector<TrackWaypoint> & out_trajectory,
    std::vector<ObstacleCluster> & out_obstacles);

  /**
   * @brief Доступ к сформированной разреженной воксельной сетке
   */
  const SparseVoxelGrid & get_voxel_grid() const { return voxel_grid_; }

private:
  void init_slice_steps();

  /**
   * @brief Оценка 3D-траектории пути по воксельной модели тоннеля
   */
  void track_trajectory(std::vector<TrackWaypoint> & trajectory);

  /**
   * @brief Проверка свободности габарита и кластеризация препятствий
   */
  void extract_obstacles(
    const std::vector<TrackWaypoint> & trajectory,
    std::vector<ObstacleCluster> & obstacles);

  VoxelTrackerConfig config_;
  SparseVoxelGrid voxel_grid_;

  std::vector<float> slice_ds_;
  int num_slices_{0};

  // Предвыделенные буферы для безаллокационной работы
  std::vector<const Voxel *> slice_voxels_;
  std::vector<float> rail_zs_;
  std::vector<float> bed_zs_;
  std::vector<float> left_wall_us_;
  std::vector<float> right_wall_us_;
  std::vector<const Voxel *> obstacle_voxel_ptrs_;
};

}  // namespace metro_voxel_tunnel_tracker
