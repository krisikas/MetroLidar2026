#include "metro_obstacle_detector/train_velocity_estimator.hpp"
#include <algorithm>
#include <numeric>

namespace metro_obstacle_detector
{

TrainVelocityEstimator::TrainVelocityEstimator(const VelocityEstimatorConfig & config)
: config_(config), current_velocity_(config.default_speed)
{
  set_config(config);
}

void TrainVelocityEstimator::set_config(const VelocityEstimatorConfig & config)
{
  config_ = config;
  const float span = config_.s_max - config_.s_min;
  num_bins_ = (span > 0.0f && config_.bin_size > 0.0f) ?
    static_cast<size_t>(std::ceil(span / config_.bin_size)) : 0;
  prev_histogram_.assign(num_bins_, 0.0f);
  curr_histogram_.assign(num_bins_, 0.0f);
  has_prev_histogram_ = false;
  current_velocity_ = config_.default_speed;
}

void TrainVelocityEstimator::reset()
{
  has_prev_histogram_ = false;
  current_velocity_ = config_.default_speed;
  std::fill(prev_histogram_.begin(), prev_histogram_.end(), 0.0f);
}

float TrainVelocityEstimator::estimate_velocity(const std::vector<Point3D> & points, float dt)
{
  if (num_bins_ == 0 || points.empty()) {
    return current_velocity_;
  }

  // 1. Build 1D longitudinal occupancy histogram along tunnel forward axis (s = -y)
  std::fill(curr_histogram_.begin(), curr_histogram_.end(), 0.0f);
  size_t valid_points = 0;

  for (const auto & pt : points) {
    const float s = -pt.y;
    // Filter tunnel side walls and overhead infrastructure (stable static reference)
    if (s >= config_.s_min && s < config_.s_max && std::abs(pt.x) >= 1.20f && pt.z >= -0.50f) {
      const size_t bin = static_cast<size_t>((s - config_.s_min) / config_.bin_size);
      if (bin < num_bins_) {
        curr_histogram_[bin] += 1.0f;
        valid_points++;
      }
    }
  }

  // Require enough static reference points to compute correlation
  if (valid_points < 200) {
    return current_velocity_;
  }

  // Normalize current histogram (zero-mean unit-variance)
  float sum = 0.0f;
  for (float val : curr_histogram_) {
    sum += val;
  }
  const float mean = sum / static_cast<float>(num_bins_);
  float sum_sq = 0.0f;
  for (float & val : curr_histogram_) {
    val -= mean;
    sum_sq += val * val;
  }
  const float std_inv = (sum_sq > 1e-4f) ? (1.0f / std::sqrt(sum_sq)) : 0.0f;
  for (float & val : curr_histogram_) {
    val *= std_inv;
  }

  if (!has_prev_histogram_ || dt <= 0.005f || dt > 1.0f) {
    prev_histogram_ = curr_histogram_;
    has_prev_histogram_ = true;
    return current_velocity_;
  }

  // 2. 1D Cross-Correlation with previous frame
  // Search shift range corresponding to speeds from -2 m/s to max_train_speed
  const int max_shift_bins = std::min(
    static_cast<int>(num_bins_ / 3),
    static_cast<int>((config_.max_train_speed * dt) / config_.bin_size) + 4);
  const int min_shift_bins = -std::min(static_cast<int>(num_bins_ / 10), static_cast<int>(0.25f / config_.bin_size));

  float best_corr = -1.0f;
  int best_shift = 0;

  for (int shift = min_shift_bins; shift <= max_shift_bins; ++shift) {
    float dot = 0.0f;
    if (shift >= 0) {
      const size_t len = num_bins_ - static_cast<size_t>(shift);
      for (size_t i = 0; i < len; ++i) {
        dot += curr_histogram_[i] * prev_histogram_[i + shift];
      }
    } else {
      const size_t offset = static_cast<size_t>(-shift);
      const size_t len = num_bins_ - offset;
      for (size_t i = 0; i < len; ++i) {
        dot += curr_histogram_[i + offset] * prev_histogram_[i];
      }
    }

    if (dot > best_corr) {
      best_corr = dot;
      best_shift = shift;
    }
  }

  // 3. Sub-bin Parabolic Peak Refinement
  float refined_shift = static_cast<float>(best_shift);
  if (best_shift > min_shift_bins && best_shift < max_shift_bins) {
    float dot_left = 0.0f;
    float dot_right = 0.0f;
    const int s_l = best_shift - 1;
    const int s_r = best_shift + 1;

    for (size_t i = 0; i < (s_l >= 0 ? num_bins_ - s_l : num_bins_ + s_l); ++i) {
      dot_left += curr_histogram_[i] * prev_histogram_[s_l >= 0 ? i + s_l : i - s_l];
    }
    for (size_t i = 0; i < (s_r >= 0 ? num_bins_ - s_r : num_bins_ + s_r); ++i) {
      dot_right += curr_histogram_[i] * prev_histogram_[s_r >= 0 ? i + s_r : i - s_r];
    }

    const float denom = 2.0f * (2.0f * best_corr - dot_left - dot_right);
    if (std::abs(denom) > 1e-4f) {
      const float delta = (dot_right - dot_left) / denom;
      refined_shift += std::clamp(delta, -0.5f, 0.5f);
    }
  }

  // 4. Velocity Calculation & Filtering
  if (best_corr >= config_.min_peak_correlation) {
    const float delta_s = refined_shift * config_.bin_size;
    float raw_v = delta_s / dt;
    if (raw_v < 0.20f) {
      raw_v = 0.0f; // Station dwell / stationary threshold
    }
    raw_v = std::clamp(raw_v, 0.0f, config_.max_train_speed);

    // Rate-of-change (acceleration) limiter
    const float max_dv = config_.max_train_accel * dt;
    const float clamped_v = std::clamp(raw_v, current_velocity_ - max_dv, current_velocity_ + max_dv);

    // Exponential smoothing filter
    current_velocity_ = config_.ema_alpha * clamped_v + (1.0f - config_.ema_alpha) * current_velocity_;
  }

  prev_histogram_ = curr_histogram_;
  return current_velocity_;
}

} // namespace metro_obstacle_detector
