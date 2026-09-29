#pragma once

#include "metro_obstacle_detector/types.hpp"
#include <vector>
#include <memory>

namespace metro_obstacle_detector
{

struct TemporalTrackerConfig
{
  uint32_t min_hits_to_confirm{2};     // Minimum frames to confirm obstacle (filters 1-frame spikes)
  uint32_t max_misses{3};              // Missed frames before deletion
  float association_dist_thresh{2.5f}; // Max association gating distance (m)
  float process_noise{0.20f};          // Kalman process noise
  float measurement_noise{0.15f};      // Kalman measurement noise
};

struct TrackedObject
{
  uint32_t id{0};
  float x{0.0f}, y{0.0f}, z{0.0f};     // State position
  float vx{0.0f}, vy{0.0f}, vz{0.0f};  // State velocity
  float p_cov[6][6]{};                 // 6x6 covariance
  float size_x{0.0f}, size_y{0.0f}, size_z{0.0f};
  float gauge_penetration{0.0f};
  ThreatLevel threat{ThreatLevel::NONE};
  ObstacleCategory category{ObstacleCategory::UNKNOWN};
  uint32_t point_count{0};
  uint32_t hits{1};
  uint32_t misses{0};
  uint32_t age{1};
  float confidence{0.40f};
  bool confirmed{false};
};

class TemporalObstacleTracker
{
public:
  explicit TemporalObstacleTracker(const TemporalTrackerConfig & config = TemporalTrackerConfig{});

  void set_config(const TemporalTrackerConfig & config);
  const TemporalTrackerConfig & get_config() const { return config_; }

  // Update tracks with new detections from current frame
  void update(
    const std::vector<Obstacle> & detections,
    float dt,
    float train_speed,
    std::vector<Obstacle> & out_confirmed_obstacles);

  void reset();

private:
  TemporalTrackerConfig config_;
  uint32_t next_track_id_{1};
  std::vector<TrackedObject> tracks_;
};

} // namespace metro_obstacle_detector
