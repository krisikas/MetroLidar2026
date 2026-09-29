#pragma once

#include "metro_obstacle_detector/types.hpp"
#include <vector>
#include <cmath>
#include <cstdint>

namespace metro_obstacle_detector
{

struct VelocityEstimatorConfig
{
  float bin_size{0.05f};             // 5 cm longitudinal bins
  float s_min{5.0f};                 // Minimum forward distance (m)
  float s_max{50.0f};                // Maximum forward distance (m)
  float max_train_speed{30.0f};      // Max plausible speed: 30 m/s (~108 km/h)
  float max_train_accel{6.0f};       // Rate limiter (m/s^2) for responsive tracking
  float min_peak_correlation{0.25f}; // Minimum normalized correlation for valid match
  float ema_alpha{0.45f};            // Exponential smoothing factor
  float default_speed{0.0f};         // Initial speed
};

class TrainVelocityEstimator
{
public:
  explicit TrainVelocityEstimator(const VelocityEstimatorConfig & config = VelocityEstimatorConfig{});

  void set_config(const VelocityEstimatorConfig & config);
  const VelocityEstimatorConfig & get_config() const { return config_; }

  // Estimates forward train velocity (m/s) from 3D LiDAR point cloud and time delta
  float estimate_velocity(const std::vector<Point3D> & points, float dt);

  float get_velocity() const { return current_velocity_; }
  bool is_moving() const { return current_velocity_ > 0.5f; }
  void reset();

private:
  VelocityEstimatorConfig config_;
  size_t num_bins_{0};
  std::vector<float> prev_histogram_;
  bool has_prev_histogram_{false};
  bool is_initialized_{false};
  float current_velocity_{0.0f};
  std::vector<float> curr_histogram_;
};

} // namespace metro_obstacle_detector
