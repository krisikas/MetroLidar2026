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
  this->declare_parameter<std::string>("lidar_topic", "/lidar_points");
  this->declare_parameter<std::string>("target_frame", "hesai_lidar");
  this->declare_parameter<std::string>("qos_reliability", "reliable");
  this->declare_parameter<float>("lookahead_distance", 150.0f);
  this->declare_parameter<float>("min_distance", 2.5f);
  this->declare_parameter<float>("slice_step", 2.5f);
  this->declare_parameter<float>("track_corridor_half_width", 0.85f);
  this->declare_parameter<float>("clearance_corridor_half_width", 1.75f);
  this->declare_parameter<float>("single_tunnel_radius", 2.15f);
  this->declare_parameter<float>("min_curve_radius", 180.0f);
  this->declare_parameter<float>("max_grade_slope", 0.035f);
  this->declare_parameter<float>("default_rail_z", -1.35f);
  this->declare_parameter<float>("gauge", 1.520f);
  this->declare_parameter<float>("temporal_alpha", 0.30f);
  this->declare_parameter<std::string>("clearance_envelope_topic", "/metro/clearance_envelope");
  this->declare_parameter<float>("carriage_width", 3.20f);
  this->declare_parameter<float>("carriage_height", 3.60f);
  this->declare_parameter<float>("carriage_wall_height", 2.70f);
  this->declare_parameter<float>("envelope_alpha", 0.18f);

  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  target_frame_ = this->get_parameter("target_frame").as_string();
  qos_reliability_ = this->get_parameter("qos_reliability").as_string();
  config_.lookahead_distance = this->get_parameter("lookahead_distance").as_double();
  config_.min_distance = this->get_parameter("min_distance").as_double();
  config_.slice_step = this->get_parameter("slice_step").as_double();
  config_.track_corridor_half_width = this->get_parameter("track_corridor_half_width").as_double();
  config_.clearance_corridor_half_width = this->get_parameter("clearance_corridor_half_width").as_double();
  config_.single_tunnel_radius = this->get_parameter("single_tunnel_radius").as_double();
  config_.min_curve_radius = this->get_parameter("min_curve_radius").as_double();
  config_.max_grade_slope = this->get_parameter("max_grade_slope").as_double();
  config_.default_rail_z = this->get_parameter("default_rail_z").as_double();
  config_.gauge = this->get_parameter("gauge").as_double();
  temporal_alpha_ = this->get_parameter("temporal_alpha").as_double();
  clearance_envelope_topic_ = this->get_parameter("clearance_envelope_topic").as_string();
  carriage_width_ = this->get_parameter("carriage_width").as_double();
  carriage_height_ = this->get_parameter("carriage_height").as_double();
  carriage_wall_height_ = this->get_parameter("carriage_wall_height").as_double();
  envelope_alpha_ = this->get_parameter("envelope_alpha").as_double();

  tracker_.set_config(config_);


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

  RCLCPP_INFO(
    this->get_logger(),
    "TunnelTrackerNode started. Topic: '%s' | QoS: %s | Target frame: '%s'",
    lidar_topic_.c_str(), qos_reliability_.c_str(), target_frame_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Publishing track geometry to: /metro/track_path and /metro/track_markers");
  RCLCPP_INFO(
    this->get_logger(),
    "Publishing clearance envelope silhouette to: '%s' (W: %.2fm, H: %.2fm, Alpha: %.2f)",
    clearance_envelope_topic_.c_str(), carriage_width_, carriage_height_, envelope_alpha_);
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
  std::vector<Point3D> points;
  points.reserve(total_points / stride);

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
    if (y > 0.5f || y < -config_.lookahead_distance - 5.0f || std::abs(x) > 8.0f) {
      continue;
    }

    float intensity = 0.0f;
    if (offset_intensity >= 0) {
      std::memcpy(&intensity, pt_ptr + offset_intensity, sizeof(float));
    }

    points.push_back(Point3D{x, y, z, intensity});
  }

  auto trajectory = tracker_.estimate_track_trajectory(points);

  if (!prev_trajectory_.empty() && prev_trajectory_.size() == trajectory.size()) {
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
    float next_y = trajectory[i].y - config_.slice_step;
    float next_z = trajectory[i].z_rail;

    if (i + 1 < trajectory.size()) {
      next_x = trajectory[i + 1].x;
      next_y = trajectory[i + 1].y;
      next_z = trajectory[i + 1].z_rail;
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

  for (const auto & wp : trajectory) {
    geometry_msgs::msg::Point p_c;
    p_c.x = wp.x;
    p_c.y = wp.y;
    p_c.z = wp.z_rail;
    center_line.points.push_back(p_c);

    geometry_msgs::msg::Point p_l;
    p_l.x = wp.x - half_gauge;
    p_l.y = wp.y;
    p_l.z = wp.z_rail;
    left_rail.points.push_back(p_l);

    geometry_msgs::msg::Point p_r;
    p_r.x = wp.x + half_gauge;
    p_r.y = wp.y;
    p_r.z = wp.z_rail;
    right_rail.points.push_back(p_r);

    geometry_msgs::msg::Point b_l;
    b_l.x = wp.left_boundary;
    b_l.y = wp.y;
    b_l.z = wp.z_rail + 1.0f;
    geometry_msgs::msg::Point b_r;
    b_r.x = wp.right_boundary;
    b_r.y = wp.y;
    b_r.z = wp.z_rail + 1.0f;
    tunnel_bounds.points.push_back(b_l);
    tunnel_bounds.points.push_back(b_r);
  }

  marker_array.markers.push_back(center_line);
  marker_array.markers.push_back(left_rail);
  marker_array.markers.push_back(right_rail);
  marker_array.markers.push_back(tunnel_bounds);

  pub_markers_->publish(marker_array);

  auto envelope_markers = create_clearance_envelope_markers(trajectory, path_msg.header);
  pub_envelope_->publish(envelope_markers);

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  if (frame_count_ <= 3) {
    RCLCPP_INFO(
      this->get_logger(),
      "[TrackTracker] Frame #%zu | Points: %zu | Poses: %zu (%lld ms) | Rail Z: %.2f m | Path & Envelope -> /metro",
      frame_count_, points.size(), trajectory.size(),
      static_cast<long long>(elapsed_ms),
      trajectory.empty() ? 0.0f : trajectory.front().z_rail);
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[TrackTracker] Frame #%zu | Points: %zu | Poses: %zu (%lld ms) | Rail Z: %.2f m | Path & Envelope -> /metro",
      frame_count_, points.size(), trajectory.size(),
      static_cast<long long>(elapsed_ms),
      trajectory.empty() ? 0.0f : trajectory.front().z_rail);
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

  const float half_width = 0.5f * carriage_width_;
  const float roof_half_w = 0.65f * half_width;
  const float wall_h = carriage_wall_height_;
  const float top_h = carriage_height_;

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

  std::vector<std::array<geometry_msgs::msg::Point, 6>> rings(trajectory.size());

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
    rings[i][0] = make_pt(wp.x - half_width * nx, wp.y - half_width * ny, wp.z_rail);
    rings[i][1] = make_pt(wp.x - half_width * nx, wp.y - half_width * ny, wp.z_rail + wall_h);
    rings[i][2] = make_pt(wp.x - roof_half_w * nx, wp.y - roof_half_w * ny, wp.z_rail + top_h);
    rings[i][3] = make_pt(wp.x + roof_half_w * nx, wp.y + roof_half_w * ny, wp.z_rail + top_h);
    rings[i][4] = make_pt(wp.x + half_width * nx, wp.y + half_width * ny, wp.z_rail + wall_h);
    rings[i][5] = make_pt(wp.x + half_width * nx, wp.y + half_width * ny, wp.z_rail);
  }

  for (size_t i = 0; i + 1 < rings.size(); ++i) {
    const auto & A = rings[i];
    const auto & B = rings[i + 1];

    for (size_t k = 0; k < 6; ++k) {
      size_t k_next = (k + 1) % 6;
      add_quad(mesh_marker, A[k], A[k_next], B[k_next], B[k]);
      add_line(wireframe_marker, A[k], B[k]);
    }

    if (i % 2 == 0) {
      for (size_t k = 0; k < 6; ++k) {
        add_line(wireframe_marker, A[k], A[(k + 1) % 6]);
      }
    }
  }

  if (!rings.empty()) {
    const auto & end_ring = rings.back();
    for (size_t k = 0; k < 6; ++k) {
      add_line(wireframe_marker, end_ring[k], end_ring[(k + 1) % 6]);
    }

    const auto & A = rings.front();
    add_triangle(front_marker, A[0], A[1], A[2]);
    add_triangle(front_marker, A[0], A[2], A[3]);
    add_triangle(front_marker, A[0], A[3], A[4]);
    add_triangle(front_marker, A[0], A[4], A[5]);

    add_line(wireframe_marker, A[1], A[4]);
    geometry_msgs::msg::Point mid_bottom = make_pt(
      0.5f * (A[0].x + A[5].x), 0.5f * (A[0].y + A[5].y), A[0].z);
    geometry_msgs::msg::Point mid_roof = make_pt(
      0.5f * (A[2].x + A[3].x), 0.5f * (A[2].y + A[3].y), A[2].z);
    add_line(wireframe_marker, mid_bottom, mid_roof);
  }

  array.markers.push_back(mesh_marker);
  array.markers.push_back(wireframe_marker);
  array.markers.push_back(front_marker);

  return array;
}

}  // namespace metro_tunnel_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(metro_tunnel_tracker::TunnelTrackerNode)


