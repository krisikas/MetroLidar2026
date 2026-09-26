#include "metro_voxel_tunnel_tracker/voxel_tunnel_tracker_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>

namespace metro_voxel_tunnel_tracker
{

VoxelTunnelTrackerNode::VoxelTunnelTrackerNode(const rclcpp::NodeOptions & options)
: Node("voxel_tunnel_tracker_node", options),
  tracker_(config_)
{
  auto make_float_desc = [](const std::string & desc, double from, double to) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    rcl_interfaces::msg::FloatingPointRange r;
    r.from_value = from;
    r.to_value = to;
    r.step = 0.0;
    d.floating_point_range.push_back(r);
    return d;
  };

  this->declare_parameter<std::string>("lidar_topic", "/lidar_points");
  this->declare_parameter<std::string>("target_frame", "hesai_lidar");
  this->declare_parameter<std::string>("qos_reliability", "reliable");

  // Параметры воксельной сетки
  this->declare_parameter("voxel_size_x", 0.10, make_float_desc("Voxel size along X (m)", 0.02, 0.50));
  this->declare_parameter("voxel_size_y", 0.20, make_float_desc("Voxel size along Y (m)", 0.05, 1.00));
  this->declare_parameter("voxel_size_z", 0.05, make_float_desc("Voxel size along Z (m)", 0.02, 0.30));
  this->declare_parameter("min_points_per_voxel", 1, rcl_interfaces::msg::ParameterDescriptor{});

  // Параметры дискретизации и трекинга пути
  this->declare_parameter("lookahead_distance", 180.0, make_float_desc("Lookahead trajectory distance (m)", 10.0, 300.0));
  this->declare_parameter("min_distance", 2.5, make_float_desc("Near blind zone in front of train (m)", 0.5, 10.0));
  this->declare_parameter("slice_step", 2.2, make_float_desc("Base longitudinal slice step (m)", 0.5, 10.0));
  this->declare_parameter("zone1_start", 35.5, make_float_desc("Mid zone start distance (m)", 10.0, 100.0));
  this->declare_parameter("zone2_start", 79.5, make_float_desc("Far zone start distance (m)", 40.0, 150.0));
  this->declare_parameter("curvature_freeze_dist", 75.0, make_float_desc("Curvature freeze distance (m)", 20.0, 150.0));
  this->declare_parameter("track_corridor_half_width", 0.85, make_float_desc("Track corridor half-width (m)", 0.4, 2.0));
  this->declare_parameter("clearance_corridor_half_width", 1.75, make_float_desc("Clearance corridor half-width (m)", 1.0, 3.0));
  this->declare_parameter("single_tunnel_radius", 2.15, make_float_desc("Nominal single tunnel radius (m)", 1.5, 4.0));
  this->declare_parameter("min_curve_radius", 160.0, make_float_desc("Minimum curve radius SP 120.13330 (m)", 50.0, 1000.0));
  this->declare_parameter("max_grade_slope", 0.035, make_float_desc("Maximum longitudinal grade slope PTE (3.5%)", 0.005, 0.10));
  this->declare_parameter("default_rail_z", -1.35, make_float_desc("Default rail Z level in lidar frame (m)", -3.0, 0.0));
  this->declare_parameter("gauge", 1.520, make_float_desc("Russian railway gauge (1520 mm)", 1.0, 2.0));
  this->declare_parameter("max_heading_slope", 0.45, make_float_desc("Maximum tangent of track yaw heading", 0.10, 1.0));
  this->declare_parameter("temporal_alpha", 0.50, make_float_desc("EMA temporal smoothing factor (0.0 - 1.0)", 0.0, 1.0));

  // Параметры УГР и рельсов
  this->declare_parameter("rail_search_tolerance", 0.16, make_float_desc("Rail search tolerance (m)", 0.05, 0.40));
  this->declare_parameter("rail_z_min_offset", -0.35, make_float_desc("Lower Z search window offset (m)", -1.0, -0.1));
  this->declare_parameter("rail_z_max_offset", 0.30, make_float_desc("Upper Z search window offset (m)", 0.1, 1.0));
  this->declare_parameter("rail_head_quantile", 0.85, make_float_desc("Rail head quantile", 0.50, 0.99));
  this->declare_parameter("track_bed_quantile", 0.85, make_float_desc("Track bed quantile", 0.50, 0.99));
  this->declare_parameter("rail_height_over_bed", 0.16, make_float_desc("Rail height over bed (m)", 0.05, 0.35));
  this->declare_parameter("min_rail_voxels", 2, rcl_interfaces::msg::ParameterDescriptor{});

  // Параметры стен
  this->declare_parameter("wall_search_min_dist", 1.25, make_float_desc("Wall search min lateral dist (m)", 0.8, 2.5));
  this->declare_parameter("wall_search_max_dist", 4.20, make_float_desc("Wall search max lateral dist (m)", 2.0, 7.0));
  this->declare_parameter("wall_z_min", 0.35, make_float_desc("Wall search min Z above rail (m)", 0.1, 1.0));
  this->declare_parameter("wall_z_max", 3.80, make_float_desc("Wall search max Z above rail (m)", 2.0, 6.0));
  this->declare_parameter("left_wall_quantile", 0.90, make_float_desc("Left wall quantile", 0.50, 0.99));
  this->declare_parameter("right_wall_quantile", 0.10, make_float_desc("Right wall quantile", 0.01, 0.50));
  this->declare_parameter("min_wall_voxels_near", 3, rcl_interfaces::msg::ParameterDescriptor{});
  this->declare_parameter("min_wall_voxels_far", 2, rcl_interfaces::msg::ParameterDescriptor{});
  this->declare_parameter("wall_near_threshold", 45.0, make_float_desc("Wall near/far distance threshold (m)", 20.0, 80.0));
  this->declare_parameter("symmetric_tunnel_min_width", 3.0, make_float_desc("Min width of symmetric single tunnel (m)", 2.0, 4.5));
  this->declare_parameter("symmetric_tunnel_max_width", 5.4, make_float_desc("Max width of symmetric single tunnel (m)", 4.0, 7.0));
  this->declare_parameter("symmetric_tunnel_wall_tolerance", 1.2, make_float_desc("Wall asymmetry tolerance for centering (m)", 0.3, 2.5));
  this->declare_parameter("wall_tracking_error_tolerance", 0.70, make_float_desc("Wall tracking error tolerance (m)", 0.2, 1.5));
  this->declare_parameter("single_wall_tolerance", 0.85, make_float_desc("Single wall tolerance (m)", 0.3, 1.8));

  // Параметры габарита подвижного состава ГОСТ 9238
  this->declare_parameter<std::string>("clearance_envelope_topic", "/metro/clearance_envelope");
  this->declare_parameter("rail_head_clearance", 0.18, make_float_desc("Clearance above rail head (m)", 0.05, 0.50));
  this->declare_parameter("undercarriage_half_width", 1.15, make_float_desc("Undercarriage half-width (m)", 0.8, 1.5));
  this->declare_parameter("contact_rail_height", 0.60, make_float_desc("Contact rail height (m)", 0.2, 1.0));
  this->declare_parameter("platform_clearance_half_width", 1.33, make_float_desc("Platform clearance half-width (m)", 1.0, 1.6));
  this->declare_parameter("platform_height", 1.25, make_float_desc("Platform height (m)", 0.8, 1.6));
  this->declare_parameter("waist_half_width", 1.37, make_float_desc("Waist half-width (m)", 1.0, 1.8));
  this->declare_parameter("carriage_wall_height", 2.60, make_float_desc("Carriage wall height (m)", 1.5, 3.5));
  this->declare_parameter("roof_half_width", 0.85, make_float_desc("Roof half-width (m)", 0.5, 1.5));
  this->declare_parameter("carriage_height", 3.60, make_float_desc("Total car height GOST 9238 (m)", 2.5, 4.5));
  this->declare_parameter("envelope_alpha", 0.18, make_float_desc("Clearance envelope transparency", 0.0, 1.0));
  this->declare_parameter("min_cluster_voxels", 3, rcl_interfaces::msg::ParameterDescriptor{});
  this->declare_parameter("cluster_distance_thresh", 0.45, make_float_desc("Cluster distance threshold (m)", 0.20, 1.0));

  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  target_frame_ = this->get_parameter("target_frame").as_string();
  qos_reliability_ = this->get_parameter("qos_reliability").as_string();

  config_.voxel_config.voxel_size_x = static_cast<float>(this->get_parameter("voxel_size_x").as_double());
  config_.voxel_config.voxel_size_y = static_cast<float>(this->get_parameter("voxel_size_y").as_double());
  config_.voxel_config.voxel_size_z = static_cast<float>(this->get_parameter("voxel_size_z").as_double());
  config_.voxel_config.min_points_per_voxel = static_cast<int>(this->get_parameter("min_points_per_voxel").as_int());

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
  temporal_alpha_ = static_cast<float>(this->get_parameter("temporal_alpha").as_double());

  config_.rail_search_tolerance = static_cast<float>(this->get_parameter("rail_search_tolerance").as_double());
  config_.rail_z_min_offset = static_cast<float>(this->get_parameter("rail_z_min_offset").as_double());
  config_.rail_z_max_offset = static_cast<float>(this->get_parameter("rail_z_max_offset").as_double());
  config_.rail_head_quantile = static_cast<float>(this->get_parameter("rail_head_quantile").as_double());
  config_.track_bed_quantile = static_cast<float>(this->get_parameter("track_bed_quantile").as_double());
  config_.rail_height_over_bed = static_cast<float>(this->get_parameter("rail_height_over_bed").as_double());
  config_.min_rail_voxels = static_cast<int>(this->get_parameter("min_rail_voxels").as_int());

  config_.wall_search_min_dist = static_cast<float>(this->get_parameter("wall_search_min_dist").as_double());
  config_.wall_search_max_dist = static_cast<float>(this->get_parameter("wall_search_max_dist").as_double());
  config_.wall_z_min = static_cast<float>(this->get_parameter("wall_z_min").as_double());
  config_.wall_z_max = static_cast<float>(this->get_parameter("wall_z_max").as_double());
  config_.left_wall_quantile = static_cast<float>(this->get_parameter("left_wall_quantile").as_double());
  config_.right_wall_quantile = static_cast<float>(this->get_parameter("right_wall_quantile").as_double());
  config_.min_wall_voxels_near = static_cast<int>(this->get_parameter("min_wall_voxels_near").as_int());
  config_.min_wall_voxels_far = static_cast<int>(this->get_parameter("min_wall_voxels_far").as_int());
  config_.wall_near_threshold = static_cast<float>(this->get_parameter("wall_near_threshold").as_double());
  config_.symmetric_tunnel_min_width = static_cast<float>(this->get_parameter("symmetric_tunnel_min_width").as_double());
  config_.symmetric_tunnel_max_width = static_cast<float>(this->get_parameter("symmetric_tunnel_max_width").as_double());
  config_.symmetric_tunnel_wall_tolerance = static_cast<float>(this->get_parameter("symmetric_tunnel_wall_tolerance").as_double());
  config_.wall_tracking_error_tolerance = static_cast<float>(this->get_parameter("wall_tracking_error_tolerance").as_double());
  config_.single_wall_tolerance = static_cast<float>(this->get_parameter("single_wall_tolerance").as_double());

  clearance_envelope_topic_ = this->get_parameter("clearance_envelope_topic").as_string();
  config_.rail_head_clearance = static_cast<float>(this->get_parameter("rail_head_clearance").as_double());
  config_.undercarriage_half_width = static_cast<float>(this->get_parameter("undercarriage_half_width").as_double());
  config_.contact_rail_height = static_cast<float>(this->get_parameter("contact_rail_height").as_double());
  config_.platform_clearance_half_width = static_cast<float>(this->get_parameter("platform_clearance_half_width").as_double());
  config_.platform_height = static_cast<float>(this->get_parameter("platform_height").as_double());
  config_.waist_half_width = static_cast<float>(this->get_parameter("waist_half_width").as_double());
  config_.carriage_wall_height = static_cast<float>(this->get_parameter("carriage_wall_height").as_double());
  config_.roof_half_width = static_cast<float>(this->get_parameter("roof_half_width").as_double());
  config_.carriage_height = static_cast<float>(this->get_parameter("carriage_height").as_double());
  envelope_alpha_ = static_cast<float>(this->get_parameter("envelope_alpha").as_double());
  config_.min_cluster_voxels = static_cast<int>(this->get_parameter("min_cluster_voxels").as_int());
  config_.cluster_distance_thresh = static_cast<float>(this->get_parameter("cluster_distance_thresh").as_double());

  tracker_.set_config(config_);

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&VoxelTunnelTrackerNode::on_parameters_set, this, std::placeholders::_1));

  rclcpp::QoS sub_qos(10);
  if (qos_reliability_ == "best_effort") {
    sub_qos = rclcpp::SensorDataQoS();
  } else {
    sub_qos.reliable().durability_volatile();
  }

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, sub_qos,
    std::bind(&VoxelTunnelTrackerNode::pointcloud_callback, this, std::placeholders::_1));

  rclcpp::QoS pub_qos(10);
  pub_qos.reliable().durability_volatile();
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/metro/track_path", pub_qos);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/metro/track_markers", pub_qos);
  pub_envelope_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(clearance_envelope_topic_, pub_qos);
  pub_obstacles_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/metro/obstacles", pub_qos);
  pub_voxels_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/metro/voxel_grid", pub_qos);

  timer_heartbeat_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&VoxelTunnelTrackerNode::heartbeat_timer_callback, this));

  points_scratch_buf_.reserve(460000);

  RCLCPP_INFO(
    this->get_logger(),
    "VoxelTunnelTrackerNode started. Topic: '%s' | Voxel: [%.2f, %.2f, %.2f]m | Horizon: %.1f m",
    lidar_topic_.c_str(), config_.voxel_config.voxel_size_x, config_.voxel_config.voxel_size_y,
    config_.voxel_config.voxel_size_z, config_.lookahead_distance);
}

void VoxelTunnelTrackerNode::heartbeat_timer_callback()
{
  if (frame_count_ == 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "[VoxelTracker: Awaiting LiDAR] Listening on '%s' (0 frames received yet).",
      lidar_topic_.c_str());
  }
}

void VoxelTunnelTrackerNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  ++frame_count_;
  const auto start_time = std::chrono::steady_clock::now();
  const std::string frame = msg->header.frame_id.empty() ? target_frame_ : msg->header.frame_id;

  int offset_x = -1, offset_y = -1, offset_z = -1, offset_intensity = -1;
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
    if (byte_idx + point_step > data_size) break;

    const uint8_t * pt_ptr = raw_data + byte_idx;
    float x, y, z;
    std::memcpy(&x, pt_ptr + offset_x, sizeof(float));
    std::memcpy(&y, pt_ptr + offset_y, sizeof(float));
    std::memcpy(&z, pt_ptr + offset_z, sizeof(float));

    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
    if (y > 0.5f || y < -config_.lookahead_distance - 12.0f || std::abs(x) > 12.0f) continue;

    float intensity = 0.0f;
    if (offset_intensity >= 0) {
      std::memcpy(&intensity, pt_ptr + offset_intensity, sizeof(float));
    }
    points_scratch_buf_.push_back(Point3D{x, y, z, intensity});
  }

  std::vector<TrackWaypoint> trajectory;
  std::vector<ObstacleCluster> obstacles;
  tracker_.process(points_scratch_buf_, trajectory, obstacles);

  // Сглаживание между кадрами (EMA)
  if (temporal_alpha_ < 0.999f && !prev_trajectory_.empty() && prev_trajectory_.size() == trajectory.size()) {
    const float alpha = temporal_alpha_;
    for (size_t i = 0; i < trajectory.size(); ++i) {
      trajectory[i].x = alpha * trajectory[i].x + (1.0f - alpha) * prev_trajectory_[i].x;
      trajectory[i].z_rail = alpha * trajectory[i].z_rail + (1.0f - alpha) * prev_trajectory_[i].z_rail;
      trajectory[i].left_boundary = alpha * trajectory[i].left_boundary + (1.0f - alpha) * prev_trajectory_[i].left_boundary;
      trajectory[i].right_boundary = alpha * trajectory[i].right_boundary + (1.0f - alpha) * prev_trajectory_[i].right_boundary;
    }
  }

  // Расчет углов ориентации yaw / pitch для кватернионов
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

  // 1. Публикация /metro/track_path
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

  // 2. Публикация /metro/track_markers
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker center_line;
  center_line.header = path_msg.header;
  center_line.ns = "voxel_track_center";
  center_line.id = 0;
  center_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  center_line.action = visualization_msgs::msg::Marker::ADD;
  center_line.scale.x = 0.08;
  center_line.color.r = 1.0f;
  center_line.color.g = 0.85f;
  center_line.color.b = 0.0f;
  center_line.color.a = 0.95f;

  visualization_msgs::msg::Marker left_rail;
  left_rail.header = path_msg.header;
  left_rail.ns = "voxel_left_rail";
  left_rail.id = 1;
  left_rail.type = visualization_msgs::msg::Marker::LINE_STRIP;
  left_rail.action = visualization_msgs::msg::Marker::ADD;
  left_rail.scale.x = 0.06;
  left_rail.color.r = 0.0f;
  left_rail.color.g = 0.95f;
  left_rail.color.b = 1.0f;
  left_rail.color.a = 0.90f;

  visualization_msgs::msg::Marker right_rail;
  right_rail.header = path_msg.header;
  right_rail.ns = "voxel_right_rail";
  right_rail.id = 2;
  right_rail.type = visualization_msgs::msg::Marker::LINE_STRIP;
  right_rail.action = visualization_msgs::msg::Marker::ADD;
  right_rail.scale.x = 0.06;
  right_rail.color.r = 0.0f;
  right_rail.color.g = 0.95f;
  right_rail.color.b = 1.0f;
  right_rail.color.a = 0.90f;

  visualization_msgs::msg::Marker tunnel_bounds;
  tunnel_bounds.header = path_msg.header;
  tunnel_bounds.ns = "voxel_tunnel_bounds";
  tunnel_bounds.id = 3;
  tunnel_bounds.type = visualization_msgs::msg::Marker::LINE_LIST;
  tunnel_bounds.action = visualization_msgs::msg::Marker::ADD;
  tunnel_bounds.scale.x = 0.04;
  tunnel_bounds.color.r = 0.3f;
  tunnel_bounds.color.g = 0.9f;
  tunnel_bounds.color.b = 0.4f;
  tunnel_bounds.color.a = 0.5f;

  const float half_gauge = 0.5f * config_.gauge;

  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto & wp = trajectory[i];

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
    }

    geometry_msgs::msg::Point p_c;
    p_c.x = wp.x; p_c.y = wp.y; p_c.z = wp.z_rail;
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

  marker_array.markers.push_back(center_line);
  marker_array.markers.push_back(left_rail);
  marker_array.markers.push_back(right_rail);
  marker_array.markers.push_back(tunnel_bounds);
  pub_markers_->publish(marker_array);

  // 3. Публикация /metro/clearance_envelope
  auto envelope_markers = create_clearance_envelope_markers(trajectory, path_msg.header);
  pub_envelope_->publish(envelope_markers);

  // 4. Публикация /metro/obstacles
  auto obstacle_markers = create_obstacle_markers(obstacles, path_msg.header);
  pub_obstacles_->publish(obstacle_markers);

  // 5. Публикация /metro/voxel_grid
  auto voxel_cloud = create_voxel_cloud_msg(tracker_.get_voxel_grid(), path_msg.header);
  pub_voxels_->publish(voxel_cloud);

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[VoxelTracker] Frame #%zu | Voxels: %zu | Waypoints: %zu | Obstacles: %zu (%lld ms)",
    frame_count_, tracker_.get_voxel_grid().size(), trajectory.size(), obstacles.size(),
    static_cast<long long>(elapsed_ms));
}

visualization_msgs::msg::MarkerArray VoxelTunnelTrackerNode::create_obstacle_markers(
  const std::vector<ObstacleCluster> & obstacles,
  const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::MarkerArray array;

  // Маркер очистки предыдущих препятствий
  visualization_msgs::msg::Marker del_marker;
  del_marker.header = header;
  del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(del_marker);

  int id = 0;
  for (const auto & obs : obstacles) {
    visualization_msgs::msg::Marker box;
    box.header = header;
    box.ns = "obstacle_box";
    box.id = id++;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = obs.center_x;
    box.pose.position.y = obs.center_y;
    box.pose.position.z = obs.center_z;
    box.pose.orientation.w = 1.0;
    box.scale.x = obs.size_x;
    box.scale.y = obs.size_y;
    box.scale.z = obs.size_z;
    box.color.r = 1.0f;
    box.color.g = 0.1f;
    box.color.b = 0.1f;
    box.color.a = 0.65f;
    array.markers.push_back(box);

    visualization_msgs::msg::Marker text;
    text.header = header;
    text.ns = "obstacle_label";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = obs.center_x;
    text.pose.position.y = obs.center_y;
    text.pose.position.z = obs.center_z + obs.size_z * 0.5f + 0.35f;
    text.scale.z = 0.35;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 0.95f;

    char buf[128];
    std::snprintf(buf, sizeof(buf), "OBSTACLE: %.1fm (%zu pts)", obs.distance, obs.total_points);
    text.text = buf;
    array.markers.push_back(text);
  }

  return array;
}

sensor_msgs::msg::PointCloud2 VoxelTunnelTrackerNode::create_voxel_cloud_msg(
  const SparseVoxelGrid & grid,
  const std_msgs::msg::Header & header) const
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(grid.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 16;
  msg.row_step = msg.width * msg.point_step;

  sensor_msgs::msg::PointField f_x, f_y, f_z, f_i;
  f_x.name = "x"; f_x.offset = 0; f_x.datatype = sensor_msgs::msg::PointField::FLOAT32; f_x.count = 1;
  f_y.name = "y"; f_y.offset = 4; f_y.datatype = sensor_msgs::msg::PointField::FLOAT32; f_y.count = 1;
  f_z.name = "z"; f_z.offset = 8; f_z.datatype = sensor_msgs::msg::PointField::FLOAT32; f_z.count = 1;
  f_i.name = "intensity"; f_i.offset = 12; f_i.datatype = sensor_msgs::msg::PointField::FLOAT32; f_i.count = 1;

  msg.fields = {f_x, f_y, f_z, f_i};
  msg.data.resize(msg.row_step);

  uint8_t * ptr = msg.data.data();
  for (const auto & v : grid.get_voxels()) {
    std::memcpy(ptr + 0, &v.x, sizeof(float));
    std::memcpy(ptr + 4, &v.y, sizeof(float));
    std::memcpy(ptr + 8, &v.z, sizeof(float));
    std::memcpy(ptr + 12, &v.mean_intensity, sizeof(float));
    ptr += 16;
  }

  return msg;
}

visualization_msgs::msg::MarkerArray VoxelTunnelTrackerNode::create_clearance_envelope_markers(
  const std::vector<TrackWaypoint> & trajectory,
  const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::MarkerArray array;
  if (trajectory.size() < 2) return array;

  visualization_msgs::msg::Marker mesh_marker;
  mesh_marker.header = header;
  mesh_marker.ns = "voxel_envelope_volume";
  mesh_marker.id = 0;
  mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  mesh_marker.action = visualization_msgs::msg::Marker::ADD;
  mesh_marker.scale.x = 1.0; mesh_marker.scale.y = 1.0; mesh_marker.scale.z = 1.0;
  mesh_marker.color.r = 0.05f; mesh_marker.color.g = 0.85f; mesh_marker.color.b = 0.65f;
  mesh_marker.color.a = envelope_alpha_;

  visualization_msgs::msg::Marker wireframe_marker;
  wireframe_marker.header = header;
  wireframe_marker.ns = "voxel_envelope_wireframe";
  wireframe_marker.id = 1;
  wireframe_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  wireframe_marker.action = visualization_msgs::msg::Marker::ADD;
  wireframe_marker.scale.x = 0.03;
  wireframe_marker.color.r = 0.0f; wireframe_marker.color.g = 1.0f; wireframe_marker.color.b = 0.80f;
  wireframe_marker.color.a = 0.70f;

  auto make_pt = [](float x, float y, float z) {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
  };

  auto add_triangle = [](visualization_msgs::msg::Marker & m,
                         const geometry_msgs::msg::Point & p1,
                         const geometry_msgs::msg::Point & p2,
                         const geometry_msgs::msg::Point & p3) {
    m.points.push_back(p1); m.points.push_back(p2); m.points.push_back(p3);
    m.points.push_back(p1); m.points.push_back(p3); m.points.push_back(p2);
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
    m.points.push_back(p1); m.points.push_back(p2);
  };

  std::vector<std::array<geometry_msgs::msg::Point, 10>> rings(trajectory.size());

  for (size_t i = 0; i < trajectory.size(); ++i) {
    float dx = 0.0f, dy = -1.0f;
    if (i + 1 < trajectory.size()) {
      dx = trajectory[i + 1].x - trajectory[i].x;
      dy = trajectory[i + 1].y - trajectory[i].y;
    } else if (i > 0) {
      dx = trajectory[i].x - trajectory[i - 1].x;
      dy = trajectory[i].y - trajectory[i - 1].y;
    }

    float norm = std::hypot(dx, dy);
    float nx = 1.0f, ny = 0.0f;
    if (norm > 1e-4f) {
      nx = -dy / norm;
      ny = dx / norm;
    }

    const auto & wp = trajectory[i];
    const float z_bot = wp.z_rail + config_.rail_head_clearance;
    const float z_cr = wp.z_rail + config_.contact_rail_height;
    const float z_plat = wp.z_rail + config_.platform_height;
    const float z_wall = wp.z_rail + config_.carriage_wall_height;
    const float z_roof = wp.z_rail + config_.carriage_height;

    const float w_under = config_.undercarriage_half_width;
    const float w_plat = config_.platform_clearance_half_width;
    const float w_waist = config_.waist_half_width;
    const float w_roof = config_.roof_half_width;

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
  }

  array.markers.push_back(mesh_marker);
  array.markers.push_back(wireframe_marker);
  return array;
}

rcl_interfaces::msg::SetParametersResult VoxelTunnelTrackerNode::on_parameters_set(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool cfg_changed = false;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    if (name == "lookahead_distance") {
      config_.lookahead_distance = static_cast<float>(param.as_double());
      cfg_changed = true;
    } else if (name == "temporal_alpha") {
      temporal_alpha_ = static_cast<float>(param.as_double());
    } else if (name == "min_cluster_voxels") {
      config_.min_cluster_voxels = static_cast<int>(param.as_int());
      cfg_changed = true;
    }
  }

  if (cfg_changed) {
    tracker_.set_config(config_);
  }
  return result;
}

}  // namespace metro_voxel_tunnel_tracker
