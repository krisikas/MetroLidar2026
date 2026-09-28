#pragma once

#include "metro_obstacle_detector/types.hpp"
#include <vector>

namespace metro_obstacle_detector
{

struct DecisionConfig
{
  float emergency_deceleration{1.30f}; // PTE emergency braking deceleration (m/s^2)
  float system_reaction_time{0.50f};   // ATP reaction and brake pipe delay (s)
  float safety_stop_margin{10.0f};     // Safety margin at stop (m)
  float default_train_speed{0.0f};     // Default operational speed 0.0 m/s (computed dynamically)
  float max_train_speed{23.6f};        // Maximum PTE speed 85 km/h (m/s)
};

class DecisionEngine
{
public:
  explicit DecisionEngine(const DecisionConfig & config = DecisionConfig{});

  void set_config(const DecisionConfig & config);
  const DecisionConfig & get_config() const { return config_; }

  // Compute safety status, braking requirements, and TTC
  SafetyDecision evaluate(
    const std::vector<Obstacle> & confirmed_obstacles,
    float current_speed);

  // Calculate required emergency braking distance at speed V
  float calculate_braking_distance(float speed_mps) const;

private:
  DecisionConfig config_;
};

} // namespace metro_obstacle_detector
