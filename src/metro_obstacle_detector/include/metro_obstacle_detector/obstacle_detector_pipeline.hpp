#pragma once

#include "metro_obstacle_detector/types.hpp"
#include "metro_obstacle_detector/cloud_parser.hpp"
#include "metro_obstacle_detector/dual_rail_tracker.hpp"
#include "metro_obstacle_detector/track_geometry_spline.hpp"
#include "metro_obstacle_detector/clearance_envelope_sdf.hpp"
#include "metro_obstacle_detector/specialized_detectors.hpp"
#include "metro_obstacle_detector/temporal_obstacle_tracker.hpp"
#include "metro_obstacle_detector/train_velocity_estimator.hpp"
#include "metro_obstacle_detector/decision_engine.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <chrono>

namespace metro_obstacle_detector
{

struct PipelineConfig
{
  CloudParserConfig parser_config;
  DualRailConfig rail_config;
  SplineTrackerConfig spline_config;
  ClearanceEnvelopeConfig envelope_config;
  DetectorConfig detector_config;
  TemporalTrackerConfig tracker_config;
  VelocityEstimatorConfig velocity_config;
  DecisionConfig decision_config;
  bool use_gpu{false};
};

class ObstacleDetectorPipeline
{
public:
  explicit ObstacleDetectorPipeline(const PipelineConfig & config = PipelineConfig{});

  void set_config(const PipelineConfig & config);
  const PipelineConfig & get_config() const { return config_; }

  // Process a raw PointCloud2 message end-to-end
  bool process_frame(
    const sensor_msgs::msg::PointCloud2 & msg,
    std::vector<FrenetWaypoint> & out_trajectory,
    DualRailMeasurement & out_rail_meas,
    SafetyDecision & out_decision,
    SystemTelemetry & out_telemetry);

  float get_estimated_train_speed() const { return velocity_estimator_.get_velocity(); }

  void reset();

private:
  PipelineConfig config_;
  CloudParser cloud_parser_;
  DualRailTracker rail_tracker_;
  TrackGeometrySpline track_spline_;
  ClearanceEnvelopeSDF envelope_sdf_;
  SpecializedDetectors specialized_detectors_;
  TemporalObstacleTracker temporal_tracker_;
  TrainVelocityEstimator velocity_estimator_;
  DecisionEngine decision_engine_;

  std::vector<Point3D> points_scratch_;
  std::chrono::steady_clock::time_point last_frame_time_;
  uint64_t last_stamp_ns_{0};
  bool first_frame_{true};
};

} // namespace metro_obstacle_detector
