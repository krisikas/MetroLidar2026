#pragma once

#include "metro_voxel_tunnel_tracker/voxel_tunnel_tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <string>

namespace metro_voxel_tunnel_tracker
{

class VoxelTunnelTrackerNode : public rclcpp::Node
{
public:
  explicit VoxelTunnelTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VoxelTunnelTrackerNode() override = default;

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void heartbeat_timer_callback();

  visualization_msgs::msg::MarkerArray create_clearance_envelope_markers(
    const std::vector<TrackWaypoint> & trajectory,
    const std_msgs::msg::Header & header) const;

  visualization_msgs::msg::MarkerArray create_obstacle_markers(
    const std::vector<ObstacleCluster> & obstacles,
    const std_msgs::msg::Header & header) const;

  sensor_msgs::msg::PointCloud2 create_voxel_cloud_msg(
    const SparseVoxelGrid & grid,
    const std_msgs::msg::Header & header) const;

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters);

  std::string lidar_topic_{"/lidar_points"};
  std::string target_frame_{"hesai_lidar"};
  std::string qos_reliability_{"reliable"};
  std::string clearance_envelope_topic_{"/metro/clearance_envelope"};
  float temporal_alpha_{0.50f};
  float envelope_alpha_{0.18f};

  VoxelTrackerConfig config_;
  VoxelTunnelTracker tracker_;

  std::vector<TrackWaypoint> prev_trajectory_;
  std::vector<Point3D> points_scratch_buf_;

  size_t frame_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_envelope_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxels_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace metro_voxel_tunnel_tracker
