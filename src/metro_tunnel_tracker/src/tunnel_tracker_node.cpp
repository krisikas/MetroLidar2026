#include "metro_tunnel_tracker/tunnel_tracker_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <array>
#include <chrono>
#include <cstring>

namespace metro_tunnel_tracker
{

TunnelTrackerNode::TunnelTrackerNode(const rclcpp::NodeOptions & options)
: Node("tunnel_tracker_node", options),
  tracker_(config_)
{
  auto make_float_desc = [](const std::string & desc) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    return d;
  };

  this->declare_parameter<std::string>("lidar_topic", "/lidar_points");
  this->declare_parameter<std::string>("target_frame", "hesai_lidar");
  this->declare_parameter<std::string>("qos_reliability", "reliable");

  this->declare_parameter("lookahead_distance", 180.0,
    make_float_desc("Lookahead trajectory distance along forward axis (m)"));
  this->declare_parameter("min_distance", 2.5,
    make_float_desc("Near blind zone in front of train / coupler (m)"));
  this->declare_parameter("slice_step", 2.2,
    make_float_desc("Longitudinal slicing step along motion axis (m)"));
  this->declare_parameter("zone1_start", 35.5,
    make_float_desc("Start distance of mid zone with 2x slice step (m)"));
  this->declare_parameter("zone2_start", 79.5,
    make_float_desc("Start distance of far zone with 4x slice step (m)"));
  this->declare_parameter("curvature_freeze_dist", 75.0,
    make_float_desc("Distance beyond which curvature updates are frozen (m)"));
  this->declare_parameter("track_corridor_half_width", 0.85,
    make_float_desc("Half-width of track bed / rail search corridor (m)"));
  this->declare_parameter("clearance_corridor_half_width", 1.75,
    make_float_desc("Half-width of train clearance corridor GOST 9238 (m)"));
  this->declare_parameter("single_tunnel_radius", 2.15,
    make_float_desc("Nominal single track tunnel half-width (m)"));
  this->declare_parameter("min_curve_radius", 160.0,
    make_float_desc("Minimum track curve radius limit SP 120.13330 (m)"));
  this->declare_parameter("max_grade_slope", 0.035,
    make_float_desc("Maximum longitudinal grade slope PTE (3.5% = 0.035)"));
  this->declare_parameter("default_rail_z", -1.35,
    make_float_desc("Nominal rail head Z level in sensor frame (m)"));
  this->declare_parameter("gauge", 1.520,
    make_float_desc("Russian railway gauge (1520 mm)"));
  this->declare_parameter("max_heading_slope", 0.45,
    make_float_desc("Maximum tangent of track yaw heading angle"));
  this->declare_parameter("temporal_alpha", 0.50,
    make_float_desc("EMA temporal smoothing factor across frames (0.0 - 1.0)"));

  // Параметры поиска рельсов и УГР:
  this->declare_parameter("rail_search_tolerance", 0.16,
    make_float_desc("Half-width search tolerance around rail head (m)"));
  this->declare_parameter("rail_z_min_offset", -0.35,
    make_float_desc("Lower Z search window offset relative to pred_z (m)"));
  this->declare_parameter("rail_z_max_offset", 0.30,
    make_float_desc("Upper Z search window offset relative to pred_z (m)"));
  this->declare_parameter("rail_head_quantile", 0.85,
    make_float_desc("Quantile for rail head height detection (0.0 - 1.0)"));
  this->declare_parameter("track_bed_quantile", 0.85,
    make_float_desc("Quantile for track bed height detection (0.0 - 1.0)"));
  this->declare_parameter("rail_height_over_bed", 0.16,
    make_float_desc("Structural height of rail head above track bed (m)"));
  this->declare_parameter("min_rail_points", 3,
    rcl_interfaces::msg::ParameterDescriptor{});

  // Параметры поиска стен и габаритов тоннеля:
  this->declare_parameter("wall_search_min_dist", 1.25,
    make_float_desc("Minimum lateral distance from centerline to tunnel wall (m)"));
  this->declare_parameter("wall_search_max_dist", 4.20,
    make_float_desc("Maximum lateral distance from centerline to tunnel wall (m)"));
  this->declare_parameter("wall_z_min", 0.35,
    make_float_desc("Lower Z boundary above rail head for wall detection (m)"));
  this->declare_parameter("wall_z_max", 3.80,
    make_float_desc("Upper Z boundary above rail head for wall detection (m)"));
  this->declare_parameter("left_wall_quantile", 0.90,
    make_float_desc("Quantile for inner surface of left wall (0.0 - 1.0)"));
  this->declare_parameter("right_wall_quantile", 0.10,
    make_float_desc("Quantile for inner surface of right wall (0.0 - 1.0)"));
  this->declare_parameter("min_wall_points_near", 4,
    rcl_interfaces::msg::ParameterDescriptor{});
  this->declare_parameter("min_wall_points_far", 3,
    rcl_interfaces::msg::ParameterDescriptor{});
  this->declare_parameter("wall_near_threshold", 45.0,
    make_float_desc("Distance threshold for wall point count requirements (m)"));
  this->declare_parameter("symmetric_tunnel_min_width", 3.0,
    make_float_desc("Minimum width of symmetric single tunnel (m)"));
  this->declare_parameter("symmetric_tunnel_max_width", 5.4,
    make_float_desc("Maximum width of symmetric single tunnel (m)"));
  this->declare_parameter("symmetric_tunnel_wall_tolerance", 1.2,
    make_float_desc("Maximum wall asymmetry tolerance for centering (m)"));
  this->declare_parameter("wall_tracking_error_tolerance", 0.70,
    make_float_desc("Error tolerance for anchoring to continuous wall (m)"));
  this->declare_parameter("single_wall_tolerance", 0.85,
    make_float_desc("Error tolerance for single-wall curve tracking (m)"));
  this->declare_parameter("max_lateral_offset", 12.0,
    make_float_desc("Maximum raw point lateral offset from sensor axis (m)"));

  // Параметры 3D-габарита вагона и зоны контроля свободности (ГОСТ 9238):
  this->declare_parameter<std::string>("clearance_envelope_topic", "/metro/clearance_envelope");
  this->declare_parameter("rail_head_clearance", 0.15,
    make_float_desc("Vertical clearance above rail head (m)"));
  this->declare_parameter("undercarriage_half_width", 1.15,
    make_float_desc("Undercarriage half-width inside third rail zone (m)"));
  this->declare_parameter("contact_rail_height", 0.60,
    make_float_desc("Upper boundary of third rail zone above rail head (m)"));
  this->declare_parameter("platform_clearance_half_width", 1.33,
    make_float_desc("Car body half-width at high platform level (m)"));
  this->declare_parameter("platform_height", 1.25,
    make_float_desc("High station platform height above rail head (m)"));
  this->declare_parameter("waist_half_width", 1.37,
    make_float_desc("Car body half-width above platform level (m)"));
  this->declare_parameter("carriage_wall_height", 2.60,
    make_float_desc("Car body vertical side wall height above rail head (m)"));
  this->declare_parameter("roof_half_width", 0.85,
    make_float_desc("Roof dome top half-width (m)"));
  this->declare_parameter("carriage_height", 3.60,
    make_float_desc("Total car height above rail head GOST 9238 (m)"));
  this->declare_parameter("envelope_alpha", 0.18,
    make_float_desc("Clearance envelope mesh transparency in Foxglove (0.0 - 1.0)"));

  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  target_frame_ = this->get_parameter("target_frame").as_string();
  qos_reliability_ = this->get_parameter("qos_reliability").as_string();
  config_.lookahead_distance = static_cast<float>(this->get_parameter("lookahead_distance").as_double());
  config_.min_distance = static_cast<float>(this->get_parameter("min_distance").as_double());
  config_.slice_step = static_cast<float>(this->get_parameter("slice_step").as_double());
  config_.zone1_start = static_cast<float>(this->get_parameter("zone1_start").as_double());
  config_.zone2_start = static_cast<float>(this->get_parameter("zone2_start").as_double());
  config_.curvature_freeze_dist = static_cast<float>(this->get_parameter("curvature_freeze_dist").as_double());
  config_.track_corridor_half_width = static_cast<float>(this->get_parameter("track_corridor_half_width").as_double());
  config_.clearance_corridor_half_width = static_cast<float>(this->get_parameter("clearance_corridor_half_width").as_double());
  config_.single_tunnel_radius = static_cast<float>(this->get_parameter("single_tunnel_radius").as_double());
  config_.min_curve_radius = static_cast<float>(this->get_parameter("min_curve_radius").as_double());
  config_.max_grade_slope = static_cast<float>(this->get_parameter("max_grade_slope").as_double());
  config_.default_rail_z = static_cast<float>(this->get_parameter("default_rail_z").as_double());
  config_.gauge = static_cast<float>(this->get_parameter("gauge").as_double());
  config_.max_heading_slope = static_cast<float>(this->get_parameter("max_heading_slope").as_double());
  config_.rail_search_tolerance = static_cast<float>(this->get_parameter("rail_search_tolerance").as_double());
  config_.rail_z_min_offset = static_cast<float>(this->get_parameter("rail_z_min_offset").as_double());
  config_.rail_z_max_offset = static_cast<float>(this->get_parameter("rail_z_max_offset").as_double());
  config_.rail_head_quantile = static_cast<float>(this->get_parameter("rail_head_quantile").as_double());
  config_.track_bed_quantile = static_cast<float>(this->get_parameter("track_bed_quantile").as_double());
  config_.rail_height_over_bed = static_cast<float>(this->get_parameter("rail_height_over_bed").as_double());
  config_.min_rail_points = static_cast<int>(this->get_parameter("min_rail_points").as_int());
  config_.wall_search_min_dist = static_cast<float>(this->get_parameter("wall_search_min_dist").as_double());
  config_.wall_search_max_dist = static_cast<float>(this->get_parameter("wall_search_max_dist").as_double());
  config_.wall_z_min = static_cast<float>(this->get_parameter("wall_z_min").as_double());
  config_.wall_z_max = static_cast<float>(this->get_parameter("wall_z_max").as_double());
  config_.left_wall_quantile = static_cast<float>(this->get_parameter("left_wall_quantile").as_double());
  config_.right_wall_quantile = static_cast<float>(this->get_parameter("right_wall_quantile").as_double());
  config_.min_wall_points_near = static_cast<int>(this->get_parameter("min_wall_points_near").as_int());
  config_.min_wall_points_far = static_cast<int>(this->get_parameter("min_wall_points_far").as_int());
  config_.wall_near_threshold = static_cast<float>(this->get_parameter("wall_near_threshold").as_double());
  config_.symmetric_tunnel_min_width = static_cast<float>(this->get_parameter("symmetric_tunnel_min_width").as_double());
  config_.symmetric_tunnel_max_width = static_cast<float>(this->get_parameter("symmetric_tunnel_max_width").as_double());
  config_.symmetric_tunnel_wall_tolerance = static_cast<float>(this->get_parameter("symmetric_tunnel_wall_tolerance").as_double());
  config_.wall_tracking_error_tolerance = static_cast<float>(this->get_parameter("wall_tracking_error_tolerance").as_double());
  config_.single_wall_tolerance = static_cast<float>(this->get_parameter("single_wall_tolerance").as_double());
  config_.max_lateral_offset = static_cast<float>(this->get_parameter("max_lateral_offset").as_double());
  temporal_alpha_ = static_cast<float>(this->get_parameter("temporal_alpha").as_double());

  clearance_envelope_topic_ = this->get_parameter("clearance_envelope_topic").as_string();
  rail_head_clearance_ = static_cast<float>(this->get_parameter("rail_head_clearance").as_double());
  undercarriage_half_width_ = static_cast<float>(this->get_parameter("undercarriage_half_width").as_double());
  contact_rail_height_ = static_cast<float>(this->get_parameter("contact_rail_height").as_double());
  platform_clearance_half_width_ = static_cast<float>(this->get_parameter("platform_clearance_half_width").as_double());
  platform_height_ = static_cast<float>(this->get_parameter("platform_height").as_double());
  waist_half_width_ = static_cast<float>(this->get_parameter("waist_half_width").as_double());
  carriage_wall_height_ = static_cast<float>(this->get_parameter("carriage_wall_height").as_double());
  roof_half_width_ = static_cast<float>(this->get_parameter("roof_half_width").as_double());
  carriage_height_ = static_cast<float>(this->get_parameter("carriage_height").as_double());
  envelope_alpha_ = static_cast<float>(this->get_parameter("envelope_alpha").as_double());

  tracker_.set_config(config_);

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TunnelTrackerNode::on_parameters_set, this, std::placeholders::_1));

  rclcpp::QoS sub_qos(10);
  if (qos_reliability_ == "best_effort") {
    sub_qos = rclcpp::SensorDataQoS();
  } else {
    sub_qos.reliable().durability_volatile();
  }

  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.incompatible_qos_callback =
    [this](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
      RCLCPP_WARN(
        this->get_logger(),
        "Incompatible QoS on '%s', policy kind: %d, count: %d",
        lidar_topic_.c_str(), info.last_policy_kind, info.total_count);
    };

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, sub_qos,
    std::bind(&TunnelTrackerNode::pointcloud_callback, this, std::placeholders::_1),
    sub_options);

  rclcpp::QoS pub_qos(10);
  pub_qos.reliable().durability_volatile();
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/metro/track_path", pub_qos);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/metro/track_markers", pub_qos);
  pub_envelope_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    clearance_envelope_topic_, pub_qos);

  timer_heartbeat_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&TunnelTrackerNode::heartbeat_timer_callback, this));

  // Pre-reserve scratch buffer
  points_scratch_buf_.reserve(460000);

  RCLCPP_INFO(
    this->get_logger(),
    "TunnelTrackerNode started. Topic: '%s' | QoS: %s | Target frame: '%s' | Horizon: %.1f m",
    lidar_topic_.c_str(), qos_reliability_.c_str(), target_frame_.c_str(), config_.lookahead_distance);
  RCLCPP_INFO(
    this->get_logger(),
    "Publishing track geometry to: /metro/track_path and /metro/track_markers");
  RCLCPP_INFO(
    this->get_logger(),
    "Publishing clearance envelope to: '%s' (Z_bot: +%.2fm, W_under: +-%.2fm, W_plat: +-%.2fm, H: %.2fm)",
    clearance_envelope_topic_.c_str(), rail_head_clearance_, undercarriage_half_width_, platform_clearance_half_width_, carriage_height_);
}

void TunnelTrackerNode::heartbeat_timer_callback()
{
  if (frame_count_ == 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "[Awaiting LiDAR] Listening on topic '%s' (0 frames received yet). "
      "Run 'ros2 bag play <bag_path> -l' in another terminal.",
      lidar_topic_.c_str());
  }
}

void TunnelTrackerNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  ++frame_count_;
  const auto start_time = std::chrono::steady_clock::now();
  const std::string frame = msg->header.frame_id.empty() ? target_frame_ : msg->header.frame_id;

  if (frame_count_ == 1) {
    RCLCPP_INFO(
      this->get_logger(),
      ">>> [FIRST LIDAR FRAME RECEIVED] Topic: '%s' | W: %u, H: %u (%zu pts) | step: %u bytes | frame: '%s'",
      lidar_topic_.c_str(), msg->width, msg->height,
      static_cast<size_t>(msg->width * msg->height), msg->point_step,
      frame.c_str());
  }

  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_intensity = -1;

  for (const auto & f : msg->fields) {
    if (f.name == "x") offset_x = static_cast<int>(f.offset);
    else if (f.name == "y") offset_y = static_cast<int>(f.offset);
    else if (f.name == "z") offset_z = static_cast<int>(f.offset);
    else if (f.name == "intensity") offset_intensity = static_cast<int>(f.offset);
  }

  if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "PointCloud2 message missing required fields (x, y, z)");
    return;
  }

  const size_t total_points = msg->width * msg->height;
  const size_t point_step = msg->point_step;
  const uint8_t * raw_data = msg->data.data();
  const size_t data_size = msg->data.size();

  const size_t stride = (total_points > 500000) ? 2 : 1;
  points_scratch_buf_.clear();
  if (points_scratch_buf_.capacity() < total_points / stride) {
    points_scratch_buf_.reserve(total_points / stride);
  }

  for (size_t i = 0; i < total_points; i += stride) {
    const size_t byte_idx = i * point_step;
    if (byte_idx + point_step > data_size) {
      break;
    }

    const uint8_t * pt_ptr = raw_data + byte_idx;
    float x, y, z;
    std::memcpy(&x, pt_ptr + offset_x, sizeof(float));
    std::memcpy(&y, pt_ptr + offset_y, sizeof(float));
    std::memcpy(&z, pt_ptr + offset_z, sizeof(float));

    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
      continue;
    }
    if (y > 0.5f || y < -config_.lookahead_distance - 12.0f || std::abs(x) > config_.max_lateral_offset) {
      continue;
    }

    float intensity = 0.0f;
    if (offset_intensity >= 0) {
      std::memcpy(&intensity, pt_ptr + offset_intensity, sizeof(float));
    }

    points_scratch_buf_.push_back(Point3D{x, y, z, intensity});
  }

  auto trajectory = tracker_.estimate_track_trajectory(points_scratch_buf_);

  if (temporal_alpha_ < 0.999f && !prev_trajectory_.empty() && prev_trajectory_.size() == trajectory.size()) {
    const float alpha = temporal_alpha_;
    for (size_t i = 0; i < trajectory.size(); ++i) {
      trajectory[i].x = alpha * trajectory[i].x + (1.0f - alpha) * prev_trajectory_[i].x;
      trajectory[i].z_rail = alpha * trajectory[i].z_rail + (1.0f - alpha) * prev_trajectory_[i].z_rail;
      trajectory[i].left_boundary = alpha * trajectory[i].left_boundary + (1.0f - alpha) * prev_trajectory_[i].left_boundary;
      trajectory[i].right_boundary = alpha * trajectory[i].right_boundary + (1.0f - alpha) * prev_trajectory_[i].right_boundary;
    }
  }

  for (size_t i = 0; i < trajectory.size(); ++i) {
    float next_x = trajectory[i].x;
    float next_y = trajectory[i].y;
    float next_z = trajectory[i].z_rail;

    if (i + 1 < trajectory.size()) {
      next_x = trajectory[i + 1].x;
      next_y = trajectory[i + 1].y;
      next_z = trajectory[i + 1].z_rail;
    } else if (i > 0) {
      next_x = trajectory[i].x + (trajectory[i].x - trajectory[i - 1].x);
      next_y = trajectory[i].y + (trajectory[i].y - trajectory[i - 1].y);
      next_z = trajectory[i].z_rail + (trajectory[i].z_rail - trajectory[i - 1].z_rail);
    } else {
      next_y = trajectory[i].y - config_.slice_step;
    }

    const float delta_x = next_x - trajectory[i].x;
    const float delta_y = next_y - trajectory[i].y;
    const float delta_z = next_z - trajectory[i].z_rail;

    trajectory[i].yaw = std::atan2(delta_x, -delta_y);
    trajectory[i].pitch = std::atan2(delta_z, -delta_y);
  }

  prev_trajectory_ = trajectory;

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = msg->header.stamp;
  path_msg.header.frame_id = frame;

  for (const auto & wp : trajectory) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = wp.z_rail;

    tf2::Quaternion q;
    q.setRPY(0.0, wp.pitch, wp.yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_msg.poses.push_back(pose);
  }
  pub_path_->publish(path_msg);


  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker center_line;
  center_line.header = path_msg.header;
  center_line.ns = "track_center";
  center_line.id = 0;
  center_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  center_line.action = visualization_msgs::msg::Marker::ADD;
  center_line.scale.x = 0.08;
  center_line.color.r = 1.0f;
  center_line.color.g = 0.82f;
  center_line.color.b = 0.0f;
  center_line.color.a = 0.95f;

  visualization_msgs::msg::Marker left_rail;
  left_rail.header = path_msg.header;
  left_rail.ns = "left_rail";
  left_rail.id = 1;
  left_rail.type = visualization_msgs::msg::Marker::LINE_STRIP;
  left_rail.action = visualization_msgs::msg::Marker::ADD;
  left_rail.scale.x = 0.06;
  left_rail.color.r = 0.0f;
  left_rail.color.g = 0.90f;
  left_rail.color.b = 1.0f;
  left_rail.color.a = 0.90f;

  visualization_msgs::msg::Marker right_rail;
  right_rail.header = path_msg.header;
  right_rail.ns = "right_rail";
  right_rail.id = 2;
  right_rail.type = visualization_msgs::msg::Marker::LINE_STRIP;
  right_rail.action = visualization_msgs::msg::Marker::ADD;
  right_rail.scale.x = 0.06;
  right_rail.color.r = 0.0f;
  right_rail.color.g = 0.90f;
  right_rail.color.b = 1.0f;
  right_rail.color.a = 0.90f;

  visualization_msgs::msg::Marker tunnel_bounds;
  tunnel_bounds.header = path_msg.header;
  tunnel_bounds.ns = "tunnel_bounds";
  tunnel_bounds.id = 3;
  tunnel_bounds.type = visualization_msgs::msg::Marker::LINE_LIST;
  tunnel_bounds.action = visualization_msgs::msg::Marker::ADD;
  tunnel_bounds.scale.x = 0.04;
  tunnel_bounds.color.r = 0.2f;
  tunnel_bounds.color.g = 0.8f;
  tunnel_bounds.color.b = 0.3f;
  tunnel_bounds.color.a = 0.5f;

  const float half_gauge = 0.5f * config_.gauge;

  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto & wp = trajectory[i];

    // Compute normal vector (nx, ny) perpendicular to track tangent at waypoint
    float nx = std::cos(wp.yaw);
    float ny = std::sin(wp.yaw);
    if (i + 1 < trajectory.size()) {
      float dx = trajectory[i + 1].x - wp.x;
      float dy = trajectory[i + 1].y - wp.y;
      float norm = std::hypot(dx, dy);
      if (norm > 1e-4f) {
        nx = -dy / norm;
        ny = dx / norm;
      }
    } else if (i > 0) {
      float dx = wp.x - trajectory[i - 1].x;
      float dy = wp.y - trajectory[i - 1].y;
      float norm = std::hypot(dx, dy);
      if (norm > 1e-4f) {
        nx = -dy / norm;
        ny = dx / norm;
      }
    }

    geometry_msgs::msg::Point p_c;
    p_c.x = wp.x;
    p_c.y = wp.y;
    p_c.z = wp.z_rail;
    center_line.points.push_back(p_c);

    geometry_msgs::msg::Point p_l;
    p_l.x = wp.x - half_gauge * nx;
    p_l.y = wp.y - half_gauge * ny;
    p_l.z = wp.z_rail;
    left_rail.points.push_back(p_l);

    geometry_msgs::msg::Point p_r;
    p_r.x = wp.x + half_gauge * nx;
    p_r.y = wp.y + half_gauge * ny;
    p_r.z = wp.z_rail;
    right_rail.points.push_back(p_r);

    // Cross-sectional slice boundary line (strictly perpendicular to track trajectory)
    float dist_l = std::abs(wp.left_boundary - wp.x);
    float dist_r = std::abs(wp.right_boundary - wp.x);
    if (dist_l < 1.0f) dist_l = config_.clearance_corridor_half_width;
    if (dist_r < 1.0f) dist_r = config_.clearance_corridor_half_width;

    geometry_msgs::msg::Point b_l;
    b_l.x = wp.x - dist_l * nx;
    b_l.y = wp.y - dist_l * ny;
    b_l.z = wp.z_rail + 1.0f;

    geometry_msgs::msg::Point b_r;
    b_r.x = wp.x + dist_r * nx;
    b_r.y = wp.y + dist_r * ny;
    b_r.z = wp.z_rail + 1.0f;

    tunnel_bounds.points.push_back(b_l);
    tunnel_bounds.points.push_back(b_r);
  }

  // 1. Прямоугольные рамки сечений (габариты срезов по высоте и ширине поиска стен/свода)
  visualization_msgs::msg::Marker slice_boxes;
  slice_boxes.header = path_msg.header;
  slice_boxes.ns = "slice_search_windows";
  slice_boxes.id = 4;
  slice_boxes.type = visualization_msgs::msg::Marker::LINE_LIST;
  slice_boxes.action = visualization_msgs::msg::Marker::ADD;
  slice_boxes.scale.x = 0.025; // толщина линии рамки
  slice_boxes.color.r = 0.0f;
  slice_boxes.color.g = 0.75f;
  slice_boxes.color.b = 1.0f;
  slice_boxes.color.a = 0.40f;

  for (size_t i = 0; i < trajectory.size(); ++i) {
    // Отображаем сечение через срез или на ключевых точках
    if (i % 2 != 0 && i != trajectory.size() - 1) {
      continue;
    }
    const auto & wp = trajectory[i];
    float nx = std::cos(wp.yaw);
    float ny = std::sin(wp.yaw);
    if (i + 1 < trajectory.size()) {
      float dx = trajectory[i + 1].x - wp.x;
      float dy = trajectory[i + 1].y - wp.y;
      float norm = std::hypot(dx, dy);
      if (norm > 1e-4f) { nx = -dy / norm; ny = dx / norm; }
    } else if (i > 0) {
      float dx = wp.x - trajectory[i - 1].x;
      float dy = wp.y - trajectory[i - 1].y;
      float norm = std::hypot(dx, dy);
      if (norm > 1e-4f) { nx = -dy / norm; ny = dx / norm; }
    }

    float dist_l = std::abs(wp.left_boundary - wp.x);
    float dist_r = std::abs(wp.right_boundary - wp.x);
    if (dist_l < 1.0f) dist_l = config_.clearance_corridor_half_width;
    if (dist_r < 1.0f) dist_r = config_.clearance_corridor_half_width;

    const float z_bottom = wp.z_rail + config_.rail_z_min_offset;
    const float z_top = wp.z_rail + std::max(2.5f, wp.ceiling_z);

    geometry_msgs::msg::Point p_bl, p_br, p_tl, p_tr;
    p_bl.x = wp.x - dist_l * nx; p_bl.y = wp.y - dist_l * ny; p_bl.z = z_bottom;
    p_br.x = wp.x + dist_r * nx; p_br.y = wp.y + dist_r * ny; p_br.z = z_bottom;
    p_tl.x = wp.x - dist_l * nx; p_tl.y = wp.y - dist_l * ny; p_tl.z = z_top;
    p_tr.x = wp.x + dist_r * nx; p_tr.y = wp.y + dist_r * ny; p_tr.z = z_top;

    // Нижняя грань
    slice_boxes.points.push_back(p_bl); slice_boxes.points.push_back(p_br);
    // Верхняя грань (потолок)
    slice_boxes.points.push_back(p_tl); slice_boxes.points.push_back(p_tr);
    // Левая стойка
    slice_boxes.points.push_back(p_bl); slice_boxes.points.push_back(p_tl);
    // Правая стойка
    slice_boxes.points.push_back(p_br); slice_boxes.points.push_back(p_tr);
  }

  // 2. Информационные 3D-метки с параметрами следования вдоль пути
  std::vector<visualization_msgs::msg::Marker> text_markers;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    // Выводим метки каждые ~15-20 метров и в конце траектории
    if (i % 6 != 0 && i != trajectory.size() - 1 && i != 0) {
      continue;
    }
    const auto & wp = trajectory[i];
    visualization_msgs::msg::Marker txt;
    txt.header = path_msg.header;
    txt.ns = "slice_diagnostics";
    txt.id = static_cast<int>(100 + i);
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x = wp.x;
    txt.pose.position.y = wp.y;
    txt.pose.position.z = wp.z_rail + 1.85f;
    txt.scale.z = 0.45; // Высота шрифта (м)

    // Индикация цвета: зеленый при высокой надежности, желто-оранжевый при дальней/меньшей надежности
    if (wp.confidence > 0.6f) {
      txt.color.r = 0.2f; txt.color.g = 1.0f; txt.color.b = 0.4f; txt.color.a = 0.95f;
    } else {
      txt.color.r = 1.0f; txt.color.g = 0.85f; txt.color.b = 0.1f; txt.color.a = 0.90f;
    }

    const float dist = std::hypot(wp.x, wp.y);
    const float radius = (std::abs(wp.curvature) > 1e-4f) ? (1.0f / std::abs(wp.curvature)) : 9999.0f;
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "[%.0fm] Conf:%.0f%% | W:%.2fm | R:%.0fm | %s%s",
      dist, wp.confidence * 100.0f,
      (std::abs(wp.right_boundary - wp.x) + std::abs(wp.left_boundary - wp.x)),
      radius,
      (wp.has_left_wall ? "L" : "-"),
      (wp.has_right_wall ? "R" : "-"));
    txt.text = buf;
    text_markers.push_back(txt);
  }

  marker_array.markers.push_back(center_line);
  marker_array.markers.push_back(left_rail);
  marker_array.markers.push_back(right_rail);
  marker_array.markers.push_back(tunnel_bounds);
  marker_array.markers.push_back(slice_boxes);
  for (auto & tm : text_markers) {
    marker_array.markers.push_back(tm);
  }

  pub_markers_->publish(marker_array);

  auto envelope_markers = create_clearance_envelope_markers(trajectory, path_msg.header);
  pub_envelope_->publish(envelope_markers);

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  std::string track_summary = "";
  if (!trajectory.empty()) {
    const auto & wp_mid = trajectory[trajectory.size() / 2];
    const auto & wp_far = trajectory.back();
    char summary_buf[256];
    std::snprintf(summary_buf, sizeof(summary_buf),
      " | Mid[%.0fm]: X=%.2fm, R=%.0fm | Far[%.0fm]: X=%.2fm, Conf=%.0f%%",
      std::hypot(wp_mid.x, wp_mid.y), wp_mid.x,
      (std::abs(wp_mid.curvature) > 1e-4f ? 1.0f / std::abs(wp_mid.curvature) : 9999.0f),
      std::hypot(wp_far.x, wp_far.y), wp_far.x, wp_far.confidence * 100.0f);
    track_summary = summary_buf;
  }

  if (frame_count_ <= 3) {
    RCLCPP_INFO(
      this->get_logger(),
      "[TrackTracker] Frame #%zu | Points: %zu | Poses: %zu (%lld ms) | Rail Z: %.2f m%s",
      frame_count_, points_scratch_buf_.size(), trajectory.size(),
      static_cast<long long>(elapsed_ms),
      trajectory.empty() ? 0.0f : trajectory.front().z_rail,
      track_summary.c_str());
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[TrackTracker] Frame #%zu | Points: %zu | Poses: %zu (%lld ms)%s",
      frame_count_, points_scratch_buf_.size(), trajectory.size(),
      static_cast<long long>(elapsed_ms), track_summary.c_str());
  }
}

visualization_msgs::msg::MarkerArray TunnelTrackerNode::create_clearance_envelope_markers(
  const std::vector<TrackWaypoint> & trajectory,
  const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::MarkerArray array;
  if (trajectory.size() < 2) {
    return array;
  }

  visualization_msgs::msg::Marker mesh_marker;
  mesh_marker.header = header;
  mesh_marker.ns = "envelope_volume";
  mesh_marker.id = 0;
  mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  mesh_marker.action = visualization_msgs::msg::Marker::ADD;
  mesh_marker.scale.x = 1.0;
  mesh_marker.scale.y = 1.0;
  mesh_marker.scale.z = 1.0;
  mesh_marker.color.r = 0.05f;
  mesh_marker.color.g = 0.85f;
  mesh_marker.color.b = 0.65f;
  mesh_marker.color.a = envelope_alpha_;

  visualization_msgs::msg::Marker wireframe_marker;
  wireframe_marker.header = header;
  wireframe_marker.ns = "envelope_wireframe";
  wireframe_marker.id = 1;
  wireframe_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  wireframe_marker.action = visualization_msgs::msg::Marker::ADD;
  wireframe_marker.scale.x = 0.03;
  wireframe_marker.color.r = 0.0f;
  wireframe_marker.color.g = 1.0f;
  wireframe_marker.color.b = 0.80f;
  wireframe_marker.color.a = 0.70f;

  visualization_msgs::msg::Marker front_marker;
  front_marker.header = header;
  front_marker.ns = "envelope_front_cab";
  front_marker.id = 2;
  front_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  front_marker.action = visualization_msgs::msg::Marker::ADD;
  front_marker.scale.x = 1.0;
  front_marker.scale.y = 1.0;
  front_marker.scale.z = 1.0;
  front_marker.color.r = 0.0f;
  front_marker.color.g = 0.95f;
  front_marker.color.b = 0.75f;
  front_marker.color.a = std::min(1.0f, envelope_alpha_ * 2.2f);

  mesh_marker.points.reserve(trajectory.size() * 60);
  wireframe_marker.points.reserve(trajectory.size() * 40);
  front_marker.points.reserve(32);

  auto make_pt = [](float x, float y, float z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };

  auto add_triangle = [](visualization_msgs::msg::Marker & m,
                         const geometry_msgs::msg::Point & p1,
                         const geometry_msgs::msg::Point & p2,
                         const geometry_msgs::msg::Point & p3) {
    m.points.push_back(p1);
    m.points.push_back(p2);
    m.points.push_back(p3);
    m.points.push_back(p1);
    m.points.push_back(p3);
    m.points.push_back(p2);
  };

  auto add_quad = [&](visualization_msgs::msg::Marker & m,
                      const geometry_msgs::msg::Point & a1,
                      const geometry_msgs::msg::Point & a2,
                      const geometry_msgs::msg::Point & b2,
                      const geometry_msgs::msg::Point & b1) {
    add_triangle(m, a1, a2, b2);
    add_triangle(m, a1, b2, b1);
  };

  auto add_line = [](visualization_msgs::msg::Marker & m,
                     const geometry_msgs::msg::Point & p1,
                     const geometry_msgs::msg::Point & p2) {
    m.points.push_back(p1);
    m.points.push_back(p2);
  };

  std::vector<std::array<geometry_msgs::msg::Point, 10>> rings(trajectory.size());

  for (size_t i = 0; i < trajectory.size(); ++i) {
    float dx = 0.0f;
    float dy = -1.0f;
    if (i + 1 < trajectory.size()) {
      dx = trajectory[i + 1].x - trajectory[i].x;
      dy = trajectory[i + 1].y - trajectory[i].y;
    } else if (i > 0) {
      dx = trajectory[i].x - trajectory[i - 1].x;
      dy = trajectory[i].y - trajectory[i - 1].y;
    }

    float norm = std::sqrt(dx * dx + dy * dy);
    float nx = 1.0f;
    float ny = 0.0f;
    if (norm > 1e-4f) {
      nx = -dy / norm;
      ny = dx / norm;
    }

    const auto & wp = trajectory[i];
    const float z_bot = wp.z_rail + rail_head_clearance_;
    const float z_cr = wp.z_rail + contact_rail_height_;
    const float z_plat = wp.z_rail + platform_height_;
    const float z_wall = wp.z_rail + carriage_wall_height_;
    const float z_roof = wp.z_rail + carriage_height_;

    const float w_under = undercarriage_half_width_;
    const float w_plat = platform_clearance_half_width_;
    const float w_waist = waist_half_width_;
    const float w_roof = roof_half_width_;

    rings[i][0] = make_pt(wp.x - w_under * nx, wp.y - w_under * ny, z_bot);
    rings[i][1] = make_pt(wp.x - w_plat * nx, wp.y - w_plat * ny, z_cr);
    rings[i][2] = make_pt(wp.x - w_plat * nx, wp.y - w_plat * ny, z_plat);
    rings[i][3] = make_pt(wp.x - w_waist * nx, wp.y - w_waist * ny, z_wall);
    rings[i][4] = make_pt(wp.x - w_roof * nx, wp.y - w_roof * ny, z_roof);
    rings[i][5] = make_pt(wp.x + w_roof * nx, wp.y + w_roof * ny, z_roof);
    rings[i][6] = make_pt(wp.x + w_waist * nx, wp.y + w_waist * ny, z_wall);
    rings[i][7] = make_pt(wp.x + w_plat * nx, wp.y + w_plat * ny, z_plat);
    rings[i][8] = make_pt(wp.x + w_plat * nx, wp.y + w_plat * ny, z_cr);
    rings[i][9] = make_pt(wp.x + w_under * nx, wp.y + w_under * ny, z_bot);
  }

  for (size_t i = 0; i + 1 < rings.size(); ++i) {
    const auto & A = rings[i];
    const auto & B = rings[i + 1];

    for (size_t k = 0; k < 10; ++k) {
      size_t k_next = (k + 1) % 10;
      add_quad(mesh_marker, A[k], A[k_next], B[k_next], B[k]);
      add_line(wireframe_marker, A[k], B[k]);
    }

    if (i % 2 == 0) {
      for (size_t k = 0; k < 10; ++k) {
        add_line(wireframe_marker, A[k], A[(k + 1) % 10]);
      }
    }
  }

  if (!rings.empty()) {
    const auto & end_ring = rings.back();
    for (size_t k = 0; k < 10; ++k) {
      add_line(wireframe_marker, end_ring[k], end_ring[(k + 1) % 10]);
    }

    const auto & A = rings.front();
    for (size_t k = 1; k <= 8; ++k) {
      add_triangle(front_marker, A[0], A[k], A[k + 1]);
    }

    add_line(wireframe_marker, A[3], A[6]);
    add_line(wireframe_marker, A[2], A[7]);
    geometry_msgs::msg::Point mid_bottom = make_pt(
      0.5f * (A[0].x + A[9].x), 0.5f * (A[0].y + A[9].y), A[0].z);
    geometry_msgs::msg::Point mid_roof = make_pt(
      0.5f * (A[4].x + A[5].x), 0.5f * (A[4].y + A[5].y), A[4].z);
    add_line(wireframe_marker, mid_bottom, mid_roof);
  }

  array.markers.push_back(mesh_marker);
  array.markers.push_back(wireframe_marker);
  array.markers.push_back(front_marker);

  return array;
}

rcl_interfaces::msg::SetParametersResult TunnelTrackerNode::on_parameters_set(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool tracker_config_changed = false;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    if (name == "lookahead_distance") {
      config_.lookahead_distance = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "min_distance") {
      config_.min_distance = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "slice_step") {
      config_.slice_step = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "zone1_start") {
      config_.zone1_start = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "zone2_start") {
      config_.zone2_start = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "curvature_freeze_dist") {
      config_.curvature_freeze_dist = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "track_corridor_half_width") {
      config_.track_corridor_half_width = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "clearance_corridor_half_width") {
      config_.clearance_corridor_half_width = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "single_tunnel_radius") {
      config_.single_tunnel_radius = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "min_curve_radius") {
      config_.min_curve_radius = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "max_grade_slope") {
      config_.max_grade_slope = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "default_rail_z") {
      config_.default_rail_z = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "gauge") {
      config_.gauge = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "max_heading_slope") {
      config_.max_heading_slope = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "rail_search_tolerance") {
      config_.rail_search_tolerance = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "rail_z_min_offset") {
      config_.rail_z_min_offset = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "rail_z_max_offset") {
      config_.rail_z_max_offset = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "rail_head_quantile") {
      config_.rail_head_quantile = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "track_bed_quantile") {
      config_.track_bed_quantile = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "rail_height_over_bed") {
      config_.rail_height_over_bed = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "min_rail_points") {
      config_.min_rail_points = static_cast<int>(param.as_int());
      tracker_config_changed = true;
    } else if (name == "wall_search_min_dist") {
      config_.wall_search_min_dist = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "wall_search_max_dist") {
      config_.wall_search_max_dist = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "wall_z_min") {
      config_.wall_z_min = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "wall_z_max") {
      config_.wall_z_max = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "left_wall_quantile") {
      config_.left_wall_quantile = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "right_wall_quantile") {
      config_.right_wall_quantile = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "min_wall_points_near") {
      config_.min_wall_points_near = static_cast<int>(param.as_int());
      tracker_config_changed = true;
    } else if (name == "min_wall_points_far") {
      config_.min_wall_points_far = static_cast<int>(param.as_int());
      tracker_config_changed = true;
    } else if (name == "wall_near_threshold") {
      config_.wall_near_threshold = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "symmetric_tunnel_min_width") {
      config_.symmetric_tunnel_min_width = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "symmetric_tunnel_max_width") {
      config_.symmetric_tunnel_max_width = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "symmetric_tunnel_wall_tolerance") {
      config_.symmetric_tunnel_wall_tolerance = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "wall_tracking_error_tolerance") {
      config_.wall_tracking_error_tolerance = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "single_wall_tolerance") {
      config_.single_wall_tolerance = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "max_lateral_offset") {
      config_.max_lateral_offset = static_cast<float>(param.as_double());
      tracker_config_changed = true;
    } else if (name == "temporal_alpha") {
      temporal_alpha_ = static_cast<float>(param.as_double());
    } else if (name == "envelope_alpha") {
      envelope_alpha_ = static_cast<float>(param.as_double());
    } else if (name == "rail_head_clearance") {
      rail_head_clearance_ = static_cast<float>(param.as_double());
    } else if (name == "undercarriage_half_width") {
      undercarriage_half_width_ = static_cast<float>(param.as_double());
    } else if (name == "contact_rail_height") {
      contact_rail_height_ = static_cast<float>(param.as_double());
    } else if (name == "platform_clearance_half_width") {
      platform_clearance_half_width_ = static_cast<float>(param.as_double());
    } else if (name == "platform_height") {
      platform_height_ = static_cast<float>(param.as_double());
    } else if (name == "waist_half_width") {
      waist_half_width_ = static_cast<float>(param.as_double());
    } else if (name == "carriage_wall_height") {
      carriage_wall_height_ = static_cast<float>(param.as_double());
    } else if (name == "roof_half_width") {
      roof_half_width_ = static_cast<float>(param.as_double());
    } else if (name == "carriage_height") {
      carriage_height_ = static_cast<float>(param.as_double());
    }
  }

  if (tracker_config_changed) {
    tracker_.set_config(config_);
    RCLCPP_INFO(
      this->get_logger(),
      "Dynamic parameter update applied: lookahead=%.1fm, step=%.2fm, R_min=%.1fm",
      config_.lookahead_distance, config_.slice_step, config_.min_curve_radius);
  }

  return result;
}

}  // namespace metro_tunnel_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(metro_tunnel_tracker::TunnelTrackerNode)


