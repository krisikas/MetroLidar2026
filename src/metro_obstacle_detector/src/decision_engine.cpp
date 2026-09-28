#include "metro_obstacle_detector/decision_engine.hpp"
#include <algorithm>
#include <cmath>

namespace metro_obstacle_detector
{

DecisionEngine::DecisionEngine(const DecisionConfig & config)
: config_(config)
{
}

void DecisionEngine::set_config(const DecisionConfig & config)
{
  config_ = config;
}

float DecisionEngine::calculate_braking_distance(float speed_mps) const
{
  const float v = std::max(0.0f, speed_mps);
  if (v < 0.20f) {
    return 0.0f; // Stationary train
  }
  const float s_reaction = v * config_.system_reaction_time;
  const float s_braking = (v * v) / (2.0f * config_.emergency_deceleration);
  return s_reaction + s_braking + config_.safety_stop_margin;
}

SafetyDecision DecisionEngine::evaluate(
  const std::vector<Obstacle> & confirmed_obstacles,
  float current_speed)
{
  SafetyDecision decision;
  const float speed = (current_speed >= 0.0f) ? current_speed : config_.default_train_speed;
  decision.train_speed = speed;
  decision.braking_distance = calculate_braking_distance(speed);
  decision.active_obstacles = confirmed_obstacles;

  if (confirmed_obstacles.empty()) {
    decision.status = SafetyStatus::CLEAR;
    decision.emergency_brake_required = false;
    decision.min_distance_to_obstacle = -1.0f;
    decision.time_to_collision = -1.0f;
    return decision;
  }

  float min_dist_hazard = 1e6f;
  float min_dist_advisory = 1e6f;

  for (const auto & obs : confirmed_obstacles) {
    if (obs.gauge_penetration <= 0.0f) {
      // Penetrating gauge = HAZARD
      if (obs.distance < min_dist_hazard) {
        min_dist_hazard = obs.distance;
      }
    } else if (obs.gauge_penetration <= 0.25f) {
      // Near gauge = ADVISORY
      if (obs.distance < min_dist_advisory) {
        min_dist_advisory = obs.distance;
      }
    }
  }

  if (min_dist_hazard < 1e5f) {
    decision.min_distance_to_obstacle = min_dist_hazard;
    if (speed >= 0.50f) {
      decision.time_to_collision = min_dist_hazard / speed;
      if (min_dist_hazard <= decision.braking_distance) {
        decision.status = SafetyStatus::EMERGENCY_BRAKE;
        decision.emergency_brake_required = true;
      } else {
        decision.status = SafetyStatus::TRACK_WARNING;
        decision.emergency_brake_required = false;
      }
    } else {
      // Stationary: track occupied ahead
      decision.time_to_collision = -1.0f;
      decision.status = SafetyStatus::TRACK_WARNING;
      decision.emergency_brake_required = false;
    }
  } else if (min_dist_advisory < 1e5f) {
    decision.min_distance_to_obstacle = min_dist_advisory;
    decision.status = SafetyStatus::NEAR_GAUGE_ADVISORY;
    decision.emergency_brake_required = false;
    decision.time_to_collision = -1.0f;
  } else {
    decision.status = SafetyStatus::CLEAR;
    decision.emergency_brake_required = false;
    decision.min_distance_to_obstacle = -1.0f;
    decision.time_to_collision = -1.0f;
  }

  return decision;
}

} // namespace metro_obstacle_detector
