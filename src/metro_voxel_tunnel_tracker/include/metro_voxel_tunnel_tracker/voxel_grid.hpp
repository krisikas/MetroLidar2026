#pragma once

#include "metro_voxel_tunnel_tracker/types.hpp"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace metro_voxel_tunnel_tracker
{

struct VoxelGridConfig
{
  float voxel_size_x{0.10f};       ///< Размер вокселя по X (м, латеральное разрешение)
  float voxel_size_y{0.20f};       ///< Размер вокселя по Y (м, продольное разрешение)
  float voxel_size_z{0.05f};       ///< Размер вокселя по Z (м, вертикальное разрешение для УГР)
  int min_points_per_voxel{1};     ///< Минимальное число точек для активации вокселя
  float max_lateral_range{10.0f};  ///< Граница поиска по X (|x| <= max)
  float max_forward_range{185.0f}; ///< Граница поиска по Y вперед (м)
  float min_z_limit{-3.5f};        ///< Нижний предел Z (м)
  float max_z_limit{3.5f};         ///< Верхний предел Z (м)
};

/**
 * @brief Разреженная 3D-воксельная сетка с индексацией вертикальных колонок (X, Y)
 */
class SparseVoxelGrid
{
public:
  explicit SparseVoxelGrid(const VoxelGridConfig & config = VoxelGridConfig());

  void set_config(const VoxelGridConfig & config);
  const VoxelGridConfig & get_config() const { return config_; }

  /**
   * @brief Очистка сетки и переиспользование предвыделенной памяти
   */
  void clear();

  /**
   * @brief Вокселизация входного сырого облака точек (O(N) вставка с вычислением центроидов)
   * @param points Сырые точки лидара
   */
  void insert_points(const std::vector<Point3D> & points);

  /**
   * @brief Получить список всех активных вокселей
   */
  const std::vector<Voxel> & get_voxels() const { return voxels_; }
  std::vector<Voxel> & get_voxels_mutable() { return voxels_; }

  /**
   * @brief Получить число активных вокселей
   */
  size_t size() const { return voxels_.size(); }

  /**
   * @brief Пространственный запрос вокселей в окрестности среза Френе
   * @param center_x Координата X центра среза
   * @param center_y Координата Y центра среза
   * @param yaw Угол рыскания касательной траектории
   * @param half_thickness Полутолщина среза вдоль касательной (|v| <= half_thickness)
   * @param max_lateral_dist Максимальное удаление по нормали (|u| <= max_lateral_dist)
   * @param out_voxels Вектор указателей на найденные воксели
   */
  void query_frenet_slice(
    float center_x, float center_y, float yaw,
    float half_thickness, float max_lateral_dist,
    std::vector<const Voxel *> & out_voxels) const;

  /**
   * @brief Кодирование 3D-индексов в уникальный 64-битный ключ
   */
  static inline uint64_t encode_key(int32_t ix, int32_t iy, int32_t iz)
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(ix + 32768)) << 32) |
           (static_cast<uint64_t>(static_cast<uint16_t>(iy + 32768)) << 16) |
           static_cast<uint64_t>(static_cast<uint16_t>(iz + 32768));
  }

  /**
   * @brief Кодирование 2D-индексов колонки (X, Y)
   */
  static inline uint64_t encode_column_key(int32_t ix, int32_t iy)
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(ix + 32768)) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(iy + 32768));
  }

  int32_t to_ix(float x) const { return static_cast<int32_t>(std::floor(x * inv_size_x_)); }
  int32_t to_iy(float y) const { return static_cast<int32_t>(std::floor(y * inv_size_y_)); }
  int32_t to_iz(float z) const { return static_cast<int32_t>(std::floor(z * inv_size_z_)); }

private:
  VoxelGridConfig config_;
  float inv_size_x_{10.0f};
  float inv_size_y_{5.0f};
  float inv_size_z_{20.0f};

  /// Хэш-таблица: 64-битный ключ -> индекс в векторе voxels_
  std::unordered_map<uint64_t, size_t> key_to_index_;

  /// Вектор всех активных вокселей сетки
  std::vector<Voxel> voxels_;

  /// Вспомогательные аккумуляторы для точного вычисления центроидов (x, y, z, intensity)
  struct VoxelAccumulator
  {
    double sum_x{0.0};
    double sum_y{0.0};
    double sum_z{0.0};
    double sum_intensity{0.0};
  };
  std::vector<VoxelAccumulator> accumulators_;

  /// Продольные корзины по Y (с шагом 2.0 м) для ускоренного поиска вокселей при запросах срезов
  float bucket_size_y_{2.0f};
  int num_y_buckets_{0};
  std::vector<std::vector<size_t>> y_bucket_indices_;
};

}  // namespace metro_voxel_tunnel_tracker
