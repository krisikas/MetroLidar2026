#include "metro_obstacle_detector/temporal_obstacle_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace metro_obstacle_detector
{

TemporalObstacleTracker::TemporalObstacleTracker(const TemporalTrackerConfig & config)
: config_(config)
{
}

void TemporalObstacleTracker::set_config(const TemporalTrackerConfig & config)
{
  config_ = config;
}

void TemporalObstacleTracker::reset()
{
  tracks_.clear();
  next_track_id_ = 1;
}

void TemporalObstacleTracker::update(
  const std::vector<Obstacle> & detections,
  float dt,
  float train_speed,
  std::vector<Obstacle> & out_confirmed_obstacles)
{
  out_confirmed_obstacles.clear();
  if (dt <= 0.001f) dt = 0.10f; // Default 10 Hz

  // 1. Predict Step (Ego-motion compensation: train travels in -Y direction)
  const float delta_y_ego = -train_speed * dt;

  for (auto & trk : tracks_) {
    // Relative position moves toward train by delta_y_ego
    trk.y -= delta_y_ego;
    trk.x += trk.vx * dt;
    trk.y += trk.vy * dt;
    trk.z += trk.vz * dt;
    trk.age += 1;
  }

  // 2. Association Step (Global Nearest Neighbor)
  const size_t n_det = detections.size();
  const size_t n_trk = tracks_.size();
  std::vector<bool> det_matched(n_det, false);
  std::vector<bool> trk_matched(n_trk, false);

  const float gate_sq = config_.association_dist_thresh * config_.association_dist_thresh;

  for (size_t i = 0; i < n_det; ++i) {
    const auto & det = detections[i];
    float min_dist_sq = gate_sq;
    int best_trk_idx = -1;

    for (size_t j = 0; j < n_trk; ++j) {
      if (trk_matched[j]) continue;

      const float dx = det.center_x - tracks_[j].x;
      const float dy = det.center_y - tracks_[j].y;
      const float dz = det.center_z - tracks_[j].z;
      const float dist_sq = dx * dx + dy * dy + dz * dz;

      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_trk_idx = static_cast<int>(j);
      }
    }

    if (best_trk_idx >= 0) {
      // 3. Update Matched Track (Kalman EMA update)
      auto & trk = tracks_[best_trk_idx];
      trk_matched[best_trk_idx] = true;
      det_matched[i] = true;

      // Update velocity estimate
      const float vx_meas = (det.center_x - trk.x) / dt;
      const float vy_meas = (det.center_y - trk.y) / dt;
      const float vz_meas = (det.center_z - trk.z) / dt;

      const float alpha_pos = 0.65f;
      const float alpha_vel = 0.35f;

      trk.x = alpha_pos * det.center_x + (1.0f - alpha_pos) * trk.x;
      trk.y = alpha_pos * det.center_y + (1.0f - alpha_pos) * trk.y;
      trk.z = alpha_pos * det.center_z + (1.0f - alpha_pos) * trk.z;

      trk.vx = alpha_vel * vx_meas + (1.0f - alpha_vel) * trk.vx;
      trk.vy = alpha_vel * vy_meas + (1.0f - alpha_vel) * trk.vy;
      trk.vz = alpha_vel * vz_meas + (1.0f - alpha_vel) * trk.vz;

      trk.size_x = 0.5f * (trk.size_x + det.size_x);
      trk.size_y = 0.5f * (trk.size_y + det.size_y);
      trk.size_z = 0.5f * (trk.size_z + det.size_z);
      trk.gauge_penetration = std::min(trk.gauge_penetration, det.gauge_penetration);
      trk.threat = det.threat;
      trk.category = det.category;
      trk.point_count = det.point_count;

      trk.hits += 1;
      trk.misses = 0;
      trk.confidence = std::min(1.0f, trk.confidence + 0.15f);

      if (trk.hits >= config_.min_hits_to_confirm) {
        trk.confirmed = true;
      }
    }
  }

  // 4. Handle Unmatched Tracks (Misses)
  for (size_t j = 0; j < n_trk; ++j) {
    if (!trk_matched[j]) {
      tracks_[j].misses += 1;
      tracks_[j].confidence = std::max(0.0f, tracks_[j].confidence - 0.20f);
    }
  }

  // 5. Initialize New Tracks for Unmatched Detections
  for (size_t i = 0; i < n_det; ++i) {
    if (!det_matched[i]) {
      TrackedObject new_trk;
      new_trk.id = next_track_id_++;
      new_trk.x = detections[i].center_x;
      new_trk.y = detections[i].center_y;
      new_trk.z = detections[i].center_z;
      new_trk.size_x = detections[i].size_x;
      new_trk.size_y = detections[i].size_y;
      new_trk.size_z = detections[i].size_z;
      new_trk.gauge_penetration = detections[i].gauge_penetration;
      new_trk.threat = detections[i].threat;
      new_trk.category = detections[i].category;
      new_trk.point_count = detections[i].point_count;
      new_trk.hits = 1;
      new_trk.misses = 0;
      new_trk.age = 1;
      // Подтверждение требует наблюдения в течение минимум 2 кадров (отсеивание случайных выбросов)
      new_trk.confirmed = (config_.min_hits_to_confirm <= 1);
      tracks_.push_back(new_trk);
    }
  }

  // 6. Prune Stale Tracks
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(), [this](const TrackedObject & trk) {
      return (trk.misses > config_.max_misses || trk.y > 2.0f);
    }),
    tracks_.end());

  // 7. Output Confirmed Tracks
  for (const auto & trk : tracks_) {
    if (trk.confirmed && trk.confidence >= 0.50f) {
      Obstacle obs;
      obs.id = trk.id;
      obs.center_x = trk.x;
      obs.center_y = trk.y;
      obs.center_z = trk.z;
      obs.distance = -trk.y;
      obs.size_x = trk.size_x;
      obs.size_y = trk.size_y;
      obs.size_z = trk.size_z;
      obs.velocity_x = trk.vx;
      obs.velocity_y = trk.vy;
      obs.velocity_z = trk.vz;
      obs.confidence = trk.confidence;
      obs.gauge_penetration = trk.gauge_penetration;
      obs.threat = trk.threat;
      obs.category = trk.category;
      obs.point_count = trk.point_count;
      obs.tracking_frames = trk.hits;
      obs.confirmed = true;

      out_confirmed_obstacles.push_back(obs);
    }
  }
}

} // namespace metro_obstacle_detector
