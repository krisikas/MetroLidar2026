#include "metro_obstacle_detector/obstacle_detector_pipeline.hpp"
#include <chrono>

namespace metro_obstacle_detector
{

ObstacleDetectorPipeline::ObstacleDetectorPipeline(const PipelineConfig & config)
: config_(config),
  cloud_parser_(config.parser_config),
  rail_tracker_(config.rail_config),
  track_spline_(config.spline_config),
  envelope_sdf_(config.envelope_config),
  specialized_detectors_(config.detector_config, config.envelope_config),
  temporal_tracker_(config.tracker_config),
  velocity_estimator_(config.velocity_config),
  decision_engine_(config.decision_config)
{
}

void ObstacleDetectorPipeline::set_config(const PipelineConfig & config)
{
  config_ = config;
  cloud_parser_.set_config(config.parser_config);
  rail_tracker_.set_config(config.rail_config);
  track_spline_.set_config(config.spline_config);
  envelope_sdf_.set_config(config.envelope_config);
  specialized_detectors_.set_config(config.detector_config);
  specialized_detectors_.set_envelope_config(config.envelope_config);
  temporal_tracker_.set_config(config.tracker_config);
  velocity_estimator_.set_config(config.velocity_config);
  decision_engine_.set_config(config.decision_config);
}

void ObstacleDetectorPipeline::reset()
{
  temporal_tracker_.reset();
  velocity_estimator_.reset();
  rail_tracker_.reset();
  first_frame_ = true;
}

bool ObstacleDetectorPipeline::process_frame(
  const sensor_msgs::msg::PointCloud2 & msg,
  std::vector<FrenetWaypoint> & out_trajectory,
  DualRailMeasurement & out_rail_meas,
  SafetyDecision & out_decision,
  SystemTelemetry & out_telemetry)
{
  const auto t0 = std::chrono::steady_clock::now();

  // 1. Ingest and parse PointCloud2 message
  size_t raw_point_count = 0;
  if (!cloud_parser_.parse(msg, points_scratch_, raw_point_count)) {
    return false;
  }

  // Calculate dt for velocity and Kalman prediction
  float dt = 0.10f;
  if (!first_frame_) {
    const std::chrono::duration<float> elapsed = t0 - last_frame_time_;
    dt = elapsed.count();
    if (dt <= 0.001f || dt > 1.0f) dt = 0.10f;
  }
  last_frame_time_ = t0;
  first_frame_ = false;

  // 2. Dual-Rail Head Extraction near cab (origin at s=0)
  out_rail_meas = rail_tracker_.estimate_slice(
    points_scratch_, 0.0f, -5.0f, config_.rail_config.default_rail_z, 0.0f);

  // Pre-calculate dual-rail slices along forward distance
  std::vector<DualRailMeasurement> rail_slices;
  rail_slices.reserve(64);
  const float lookahead = config_.spline_config.lookahead_distance;
  const float step = config_.spline_config.base_step;

  float last_z = out_rail_meas.crown_z;
  float last_x = out_rail_meas.track_center_x;

  for (float d = 2.0f; d < std::min(70.0f, lookahead); d += step) {
    auto meas = rail_tracker_.estimate_slice(points_scratch_, last_x, -d, last_z, 0.0f);
    if (meas.confidence >= 0.50f) {
      last_z = meas.crown_z;
      last_x = meas.track_center_x;
    }
    rail_slices.push_back(meas);
  }

  // 3. Build 3D Frenet Track Ribbon
  track_spline_.build_trajectory(points_scratch_, rail_slices, out_trajectory);
  rail_tracker_.smooth_profile(out_trajectory);

  // 4. Specialized Obstacle Detection (Multi-scale: bulk, rail-mounted, suspended)
  std::vector<Obstacle> raw_candidates;
  specialized_detectors_.detect_obstacles(points_scratch_, out_trajectory, raw_candidates);

  // 5. Dynamic Train Velocity & Temporal Multi-Target Tracking
  const float estimated_speed = velocity_estimator_.estimate_velocity(points_scratch_, dt);
  const float effective_speed = (config_.decision_config.default_train_speed > 0.1f) ?
    config_.decision_config.default_train_speed : estimated_speed;

  std::vector<Obstacle> confirmed_obstacles;
  temporal_tracker_.update(raw_candidates, dt, effective_speed, confirmed_obstacles);

  // 6. Decision Engine & Threat Level Assessment
  out_decision = decision_engine_.evaluate(confirmed_obstacles, effective_speed);
  out_decision.train_speed = effective_speed;

  // 7. Telemetry & Performance Metrics
  const auto t1 = std::chrono::steady_clock::now();
  const std::chrono::duration<float, std::milli> proc_time = t1 - t0;

  out_telemetry.processing_time_ms = proc_time.count();
  out_telemetry.fps = (proc_time.count() > 0.0f) ? (1000.0f / proc_time.count()) : 0.0f;
  out_telemetry.input_points = static_cast<uint32_t>(raw_point_count);
  out_telemetry.valid_points = static_cast<uint32_t>(points_scratch_.size());
  out_telemetry.candidate_clusters = static_cast<uint32_t>(raw_candidates.size());
  out_telemetry.confirmed_obstacles = static_cast<uint32_t>(confirmed_obstacles.size());
  out_telemetry.active_tracks = static_cast<uint32_t>(out_decision.active_obstacles.size());
  out_telemetry.gpu_accelerated = config_.use_gpu;

  return true;
}

} // namespace metro_obstacle_detector
