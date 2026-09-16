#pragma once

#include "metro_tunnel_tracker/rail_geometry_tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>

namespace metro_tunnel_tracker
{

class TunnelTrackerNode : public rclcpp::Node
{
public:
  explicit TunnelTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void heartbeat_timer_callback();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_envelope_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;

  visualization_msgs::msg::MarkerArray create_clearance_envelope_markers(
    const std::vector<TrackWaypoint> & trajectory,
    const std_msgs::msg::Header & header) const;

  TrackerConfig config_;
  RailGeometryTracker tracker_;
  std::string lidar_topic_;
  std::string target_frame_;
  std::string qos_reliability_;
  std::string clearance_envelope_topic_{"/metro/clearance_envelope"};
  float carriage_width_{3.20f};
  float carriage_height_{3.60f};
  float carriage_wall_height_{2.70f};
  float envelope_alpha_{0.18f};
  float temporal_alpha_{0.30f};
  size_t frame_count_{0};
  std::vector<TrackWaypoint> prev_trajectory_;
};

}  // namespace metro_tunnel_tracker

