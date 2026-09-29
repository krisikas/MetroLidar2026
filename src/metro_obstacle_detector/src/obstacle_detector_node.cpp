#include "metro_obstacle_detector/obstacle_detector_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace metro_obstacle_detector
{

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions & options)
: Node("metro_obstacle_detector_node", options),
  pipeline_(config_)
{
  // Declare parameters
  this->declare_parameter<std::string>("lidar_topic", "/lidar_points");
  this->declare_parameter<std::string>("target_frame", "hesai_lidar");
  this->declare_parameter<std::string>("qos_reliability", "reliable");
  this->declare_parameter<bool>("use_gpu", false);
  this->declare_parameter<bool>("publish_markers", true);
  this->declare_parameter<bool>("publish_envelope", true);
  this->declare_parameter<double>("train_speed_mps", 0.0);
  this->declare_parameter<double>("lookahead_distance", 150.0);
  this->declare_parameter<double>("min_distance", 2.0);
  this->declare_parameter<double>("base_step", 2.0);
  this->declare_parameter<double>("min_curve_radius", 160.0);
  this->declare_parameter<double>("gauge", 1.520);
  this->declare_parameter<double>("nominal_tor_height", 1.075);
  this->declare_parameter<double>("emergency_deceleration", 1.30);
  this->declare_parameter<double>("system_reaction_time", 0.50);

  // Retrieve parameters
  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  target_frame_ = this->get_parameter("target_frame").as_string();
  qos_reliability_ = this->get_parameter("qos_reliability").as_string();
  publish_markers_ = this->get_parameter("publish_markers").as_bool();
  publish_envelope_ = this->get_parameter("publish_envelope").as_bool();
  current_train_speed_ = static_cast<float>(this->get_parameter("train_speed_mps").as_double());

  config_.spline_config.lookahead_distance = static_cast<float>(this->get_parameter("lookahead_distance").as_double());
  config_.spline_config.min_distance = static_cast<float>(this->get_parameter("min_distance").as_double());
  config_.spline_config.base_step = static_cast<float>(this->get_parameter("base_step").as_double());
  config_.spline_config.min_curve_radius = static_cast<float>(this->get_parameter("min_curve_radius").as_double());
  config_.detector_config.max_detection_range = config_.spline_config.lookahead_distance;
  config_.detector_config.min_detection_range = config_.spline_config.min_distance;
  config_.rail_config.gauge = static_cast<float>(this->get_parameter("gauge").as_double());
  config_.rail_config.nominal_tor_height = static_cast<float>(this->get_parameter("nominal_tor_height").as_double());
  config_.rail_config.default_rail_z = -config_.rail_config.nominal_tor_height;
  config_.decision_config.default_train_speed = current_train_speed_;
  config_.decision_config.emergency_deceleration = static_cast<float>(this->get_parameter("emergency_deceleration").as_double());
  config_.decision_config.system_reaction_time = static_cast<float>(this->get_parameter("system_reaction_time").as_double());
  config_.use_gpu = this->get_parameter("use_gpu").as_bool();

  pipeline_.set_config(config_);

  // QoS configurations
  // 1. Cloud subscription QoS (SensorData profile: BestEffort, Volatile, Depth 5)
  rclcpp::QoS qos_sub = rclcpp::SensorDataQoS();
  if (qos_reliability_ == "reliable") {
    qos_sub.reliable();
  } else {
    qos_sub.best_effort();
  }

  // 2. Obstacles & Critical Telemetry QoS (Reliable + TransientLocal for latched safety delivery)
  rclcpp::QoS qos_obstacles(rclcpp::KeepLast(10));
  qos_obstacles.reliable();
  qos_obstacles.transient_local();

  // 3. Track Profile & Path QoS (Reliable + TransientLocal)
  rclcpp::QoS qos_track(rclcpp::KeepLast(5));
  qos_track.reliable();
  qos_track.transient_local();

  // 4. Standard diagnostics QoS
  rclcpp::QoS qos_diag(rclcpp::KeepLast(10));
  qos_diag.reliable();

  // 5. Visualization QoS (BestEffort for 3D marker stream)
  rclcpp::QoS qos_vis(rclcpp::KeepLast(5));
  qos_vis.best_effort();

  // Create publishers
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/metro/track_path", qos_track);
  pub_track_profile_ = this->create_publisher<metro_obstacle_detector_interfaces::msg::TrackProfile>(
    "/metro/track_profile", qos_track);
  pub_obstacles_ = this->create_publisher<metro_obstacle_detector_interfaces::msg::ObstacleArray>(
    "/metro/obstacles", qos_obstacles);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/metro/markers", qos_vis);
  pub_envelope_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/metro/clearance_envelope", qos_vis);
  pub_telemetry_ = this->create_publisher<metro_obstacle_detector_interfaces::msg::SystemHealth>(
    "/metro/telemetry", qos_diag);
  pub_speed_ = this->create_publisher<std_msgs::msg::Float32>(
    "/metro/train_speed", qos_diag);

  // Create subscriptions
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, qos_sub,
    std::bind(&ObstacleDetectorNode::pointcloud_callback, this, std::placeholders::_1));

  sub_speed_ = this->create_subscription<std_msgs::msg::Float32>(
    "/metro/train_speed_override", qos_diag,
    std::bind(&ObstacleDetectorNode::speed_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "Metro Obstacle Detector initialized! Topic: %s, Frame: %s, Lookahead: %.1f m",
    lidar_topic_.c_str(), target_frame_.c_str(), config_.spline_config.lookahead_distance);
}

void ObstacleDetectorNode::speed_callback(const std_msgs::msg::Float32::ConstSharedPtr msg)
{
  if (msg->data >= 0.0f) {
    current_train_speed_ = msg->data;
  }
}

void ObstacleDetectorNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  ++frame_count_;

  std::vector<FrenetWaypoint> trajectory;
  DualRailMeasurement rail_meas;
  SafetyDecision decision;
  SystemTelemetry telemetry;

  if (!pipeline_.process_frame(*msg, trajectory, rail_meas, decision, telemetry)) {
    return;
  }

  const std_msgs::msg::Header header = msg->header;

  // Publish track path
  publish_path(header, trajectory);

  // Publish detailed track profile
  publish_track_profile(header, trajectory, rail_meas);

  // Publish obstacle array and safety decisions
  publish_obstacles(header, decision);

  // Periodic and event-triggered terminal logging
  if (frame_count_ % 10 == 0 || !decision.active_obstacles.empty()) {
    if (decision.active_obstacles.empty()) {
      RCLCPP_INFO(this->get_logger(),
        "[Frame %lu] Track: %zu waypoints | Status: CLEAR | 0 obstacles in clearance envelope",
        frame_count_, trajectory.size());
    } else {
      RCLCPP_WARN(this->get_logger(),
        "[Frame %lu] Track: %zu waypoints | Status: %s | Obstacles: %zu (Closest: %.1f m)",
        frame_count_, trajectory.size(),
        decision.emergency_brake_required ? "EMERGENCY_BRAKE" : "WARNING",
        decision.active_obstacles.size(), decision.min_distance_to_obstacle);
    }
  }

  // Publish train speed
  if (pub_speed_) {
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = decision.train_speed;
    pub_speed_->publish(speed_msg);
  }

  // Publish visualization markers
  if (publish_markers_) {
    publish_markers(header, decision, trajectory);
  }

  // Publish clearance envelope
  if (publish_envelope_) {
    publish_envelope(header, trajectory);
  }

  // Publish system telemetry
  publish_telemetry(header, telemetry);
}

void ObstacleDetectorNode::publish_path(
  const std_msgs::msg::Header & header,
  const std::vector<FrenetWaypoint> & waypoints)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(waypoints.size());

  for (const auto & wp : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = wp.z;

    tf2::Quaternion q;
    q.setRPY(wp.roll, wp.pitch, wp.yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path.poses.push_back(pose);
  }
  pub_path_->publish(path);
}

void ObstacleDetectorNode::publish_track_profile(
  const std_msgs::msg::Header & header,
  const std::vector<FrenetWaypoint> & waypoints,
  const DualRailMeasurement & rail_meas)
{
  metro_obstacle_detector_interfaces::msg::TrackProfile prof;
  prof.header = header;
  prof.rail_head_z = rail_meas.crown_z;
  prof.grade_slope = (waypoints.size() > 1) ? waypoints.front().pitch : 0.0f;
  prof.curvature = (waypoints.size() > 1) ? waypoints.front().curvature : 0.0f;
  prof.roll_cant = rail_meas.roll_angle;
  prof.track_confidence = rail_meas.confidence;

  const float half_g = 0.5f * config_.rail_config.gauge;

  for (const auto & wp : waypoints) {
    geometry_msgs::msg::Point p_center;
    p_center.x = wp.x;
    p_center.y = wp.y;
    p_center.z = wp.z;
    prof.waypoints.push_back(p_center);

    const float cos_y = std::cos(wp.yaw);
    const float sin_y = std::sin(wp.yaw);

    geometry_msgs::msg::Point p_left, p_right;
    p_left.x = wp.x - half_g * cos_y;
    p_left.y = wp.y - half_g * sin_y;
    p_left.z = wp.z;
    prof.left_rail.push_back(p_left);

    p_right.x = wp.x + half_g * cos_y;
    p_right.y = wp.y + half_g * sin_y;
    p_right.z = wp.z;
    prof.right_rail.push_back(p_right);
  }

  pub_track_profile_->publish(prof);
}

void ObstacleDetectorNode::publish_obstacles(
  const std_msgs::msg::Header & header,
  const SafetyDecision & decision)
{
  metro_obstacle_detector_interfaces::msg::ObstacleArray arr;
  arr.header = header;
  arr.train_speed = decision.train_speed;
  arr.min_distance_to_obstacle = decision.min_distance_to_obstacle;
  arr.time_to_collision = decision.time_to_collision;
  arr.emergency_brake_required = decision.emergency_brake_required;
  arr.emergency_braking_distance = decision.braking_distance;
  arr.alert_status = static_cast<uint8_t>(decision.status);

  arr.obstacles.reserve(decision.active_obstacles.size());
  for (const auto & obs : decision.active_obstacles) {
    metro_obstacle_detector_interfaces::msg::Obstacle o;
    o.id = obs.id;
    o.distance = obs.distance;
    o.position.x = obs.center_x;
    o.position.y = obs.center_y;
    o.position.z = obs.center_z;
    o.size.x = obs.size_x;
    o.size.y = obs.size_y;
    o.size.z = obs.size_z;
    o.velocity.x = obs.velocity_x;
    o.velocity.y = obs.velocity_y;
    o.velocity.z = obs.velocity_z;
    o.confidence = obs.confidence;
    o.gauge_penetration = obs.gauge_penetration;
    o.threat_level = static_cast<uint8_t>(obs.threat);
    o.obstacle_type = static_cast<uint8_t>(obs.category);
    o.point_count = obs.point_count;
    o.tracking_frames = obs.tracking_frames;

    arr.obstacles.push_back(o);
  }

  pub_obstacles_->publish(arr);
}

void ObstacleDetectorNode::publish_markers(
  const std_msgs::msg::Header & header,
  const SafetyDecision & decision,
  const std::vector<FrenetWaypoint> & waypoints)
{
  visualization_msgs::msg::MarkerArray markers;

  // 1. Clear previous markers
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  int marker_id = 0;

  // 2. Obstacle 3D Bounding Boxes
  for (const auto & obs : decision.active_obstacles) {
    visualization_msgs::msg::Marker bbox;
    bbox.header = header;
    bbox.ns = "obstacles_bbox";
    bbox.id = ++marker_id;
    bbox.type = visualization_msgs::msg::Marker::CUBE;
    bbox.action = visualization_msgs::msg::Marker::ADD;

    bbox.pose.position.x = obs.center_x;
    bbox.pose.position.y = obs.center_y;
    bbox.pose.position.z = obs.center_z;
    bbox.pose.orientation.w = 1.0;

    bbox.scale.x = obs.size_x;
    bbox.scale.y = obs.size_y;
    bbox.scale.z = obs.size_z;

    // Color code based on threat
    if (obs.threat == ThreatLevel::CRITICAL) {
      bbox.color.r = 1.0f; bbox.color.g = 0.0f; bbox.color.b = 0.0f; bbox.color.a = 0.85f; // RED
    } else if (obs.threat == ThreatLevel::WARNING) {
      bbox.color.r = 1.0f; bbox.color.g = 0.5f; bbox.color.b = 0.0f; bbox.color.a = 0.80f; // ORANGE
    } else if (obs.threat == ThreatLevel::CAUTION) {
      bbox.color.r = 1.0f; bbox.color.g = 1.0f; bbox.color.b = 0.0f; bbox.color.a = 0.70f; // YELLOW
    } else {
      bbox.color.r = 0.0f; bbox.color.g = 1.0f; bbox.color.b = 0.0f; bbox.color.a = 0.50f; // GREEN
    }
    markers.markers.push_back(bbox);

    // Text label: ID, distance, threat
    visualization_msgs::msg::Marker text;
    text.header = header;
    text.ns = "obstacles_text";
    text.id = ++marker_id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = obs.center_x;
    text.pose.position.y = obs.center_y;
    text.pose.position.z = obs.center_z + 0.5f * obs.size_z + 0.40f;
    text.scale.z = 0.45;
    text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;

    char buf[128];
    std::snprintf(buf, sizeof(buf), "#%u: %.1fm (conf: %.0f%%, sdf: %+.2fm)",
      obs.id, obs.distance, obs.confidence * 100.0f, obs.gauge_penetration);
    text.text = buf;
    markers.markers.push_back(text);
  }

  // 3. Rail Lines
  if (waypoints.size() >= 2) {
    visualization_msgs::msg::Marker rails;
    rails.header = header;
    rails.ns = "rail_threads";
    rails.id = ++marker_id;
    rails.type = visualization_msgs::msg::Marker::LINE_LIST;
    rails.action = visualization_msgs::msg::Marker::ADD;
    rails.scale.x = 0.06;
    rails.color.r = 0.8f; rails.color.g = 0.8f; rails.color.b = 0.8f; rails.color.a = 0.9f;

    const float half_g = 0.5f * config_.rail_config.gauge;

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      const auto & w1 = waypoints[i];
      const auto & w2 = waypoints[i + 1];

      // Left rail segment
      geometry_msgs::msg::Point p1_l, p2_l;
      p1_l.x = w1.x - half_g * std::cos(w1.yaw);
      p1_l.y = w1.y - half_g * std::sin(w1.yaw);
      p1_l.z = w1.z;
      p2_l.x = w2.x - half_g * std::cos(w2.yaw);
      p2_l.y = w2.y - half_g * std::sin(w2.yaw);
      p2_l.z = w2.z;
      rails.points.push_back(p1_l);
      rails.points.push_back(p2_l);

      // Right rail segment
      geometry_msgs::msg::Point p1_r, p2_r;
      p1_r.x = w1.x + half_g * std::cos(w1.yaw);
      p1_r.y = w1.y + half_g * std::sin(w1.yaw);
      p1_r.z = w1.z;
      p2_r.x = w2.x + half_g * std::cos(w2.yaw);
      p2_r.y = w2.y + half_g * std::sin(w2.yaw);
      p2_r.z = w2.z;
      rails.points.push_back(p1_r);
      rails.points.push_back(p2_r);
    }
    markers.markers.push_back(rails);
  }

  pub_markers_->publish(markers);
}

void ObstacleDetectorNode::publish_envelope(
  const std_msgs::msg::Header & header,
  const std::vector<FrenetWaypoint> & waypoints)
{
  if (waypoints.size() < 2) return;

  visualization_msgs::msg::MarkerArray env_markers;
  visualization_msgs::msg::Marker ribs;
  ribs.header = header;
  ribs.ns = "clearance_contour_ribs";
  ribs.id = 1;
  ribs.type = visualization_msgs::msg::Marker::LINE_LIST;
  ribs.action = visualization_msgs::msg::Marker::ADD;
  ribs.scale.x = 0.03;
  ribs.color.r = 0.0f; ribs.color.g = 0.8f; ribs.color.b = 1.0f; ribs.color.a = 0.35f; // Cyan

  ClearanceEnvelopeSDF sdf;
  const auto contour = sdf.get_polygon_contour();

  // Draw clearance ribs every 5 waypoints
  for (size_t w_idx = 0; w_idx < waypoints.size(); w_idx += 3) {
    const auto & wp = waypoints[w_idx];
    const float cos_y = std::cos(wp.yaw);
    const float sin_y = std::sin(wp.yaw);

    for (size_t c = 0; c + 1 < contour.size(); ++c) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = wp.x + contour[c].first * cos_y;
      p1.y = wp.y + contour[c].first * sin_y;
      p1.z = wp.z + contour[c].second;

      p2.x = wp.x + contour[c + 1].first * cos_y;
      p2.y = wp.y + contour[c + 1].first * sin_y;
      p2.z = wp.z + contour[c + 1].second;

      ribs.points.push_back(p1);
      ribs.points.push_back(p2);
    }
  }

  env_markers.markers.push_back(ribs);
  pub_envelope_->publish(env_markers);
}

void ObstacleDetectorNode::publish_telemetry(
  const std_msgs::msg::Header & header,
  const SystemTelemetry & telemetry)
{
  metro_obstacle_detector_interfaces::msg::SystemHealth health;
  health.header = header;
  health.processing_time_ms = telemetry.processing_time_ms;
  health.fps = telemetry.fps;
  health.input_points = telemetry.input_points;
  health.valid_points = telemetry.valid_points;
  health.voxel_count = telemetry.voxel_count;
  health.candidate_clusters = telemetry.candidate_clusters;
  health.confirmed_obstacles = telemetry.confirmed_obstacles;
  health.active_tracks = telemetry.active_tracks;
  health.gpu_accelerated = telemetry.gpu_accelerated;

  pub_telemetry_->publish(health);
}

} // namespace metro_obstacle_detector
