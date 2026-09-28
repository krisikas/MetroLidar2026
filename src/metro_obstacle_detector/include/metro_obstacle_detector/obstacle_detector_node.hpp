#pragma once

#include "metro_obstacle_detector/types.hpp"
#include "metro_obstacle_detector/obstacle_detector_pipeline.hpp"
#include "metro_obstacle_detector_interfaces/msg/obstacle.hpp"
#include "metro_obstacle_detector_interfaces/msg/obstacle_array.hpp"
#include "metro_obstacle_detector_interfaces/msg/track_profile.hpp"
#include "metro_obstacle_detector_interfaces/msg/system_health.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <string>

namespace metro_obstacle_detector
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void speed_callback(const std_msgs::msg::Float32::ConstSharedPtr msg);

  void publish_path(const std_msgs::msg::Header & header, const std::vector<FrenetWaypoint> & waypoints);
  void publish_track_profile(const std_msgs::msg::Header & header, const std::vector<FrenetWaypoint> & waypoints, const DualRailMeasurement & rail_meas);
  void publish_obstacles(const std_msgs::msg::Header & header, const SafetyDecision & decision);
  void publish_markers(const std_msgs::msg::Header & header, const SafetyDecision & decision, const std::vector<FrenetWaypoint> & waypoints);
  void publish_envelope(const std_msgs::msg::Header & header, const std::vector<FrenetWaypoint> & waypoints);
  void publish_telemetry(const std_msgs::msg::Header & header, const SystemTelemetry & telemetry);

  ObstacleDetectorPipeline pipeline_;
  PipelineConfig config_;

  std::string lidar_topic_{"/lidar_points"};
  std::string target_frame_{"hesai_lidar"};
  std::string qos_reliability_{"reliable"};
  bool publish_markers_{true};
  bool publish_envelope_{true};
  float current_train_speed_{15.0f};
  uint64_t frame_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<metro_obstacle_detector_interfaces::msg::TrackProfile>::SharedPtr pub_track_profile_;
  rclcpp::Publisher<metro_obstacle_detector_interfaces::msg::ObstacleArray>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_envelope_;
  rclcpp::Publisher<metro_obstacle_detector_interfaces::msg::SystemHealth>::SharedPtr pub_telemetry_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;
};

} // namespace metro_obstacle_detector
