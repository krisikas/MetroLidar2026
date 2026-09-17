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
  std::string lidar_topic_{"/lidar_points"};
  std::string target_frame_{"hesai_lidar"};
  std::string qos_reliability_{"reliable"};
  std::string clearance_envelope_topic_{"/metro/clearance_envelope"};

  // Геометрия габарита вагона и зоны контроля свободности (ГОСТ 9238):
  float rail_head_clearance_{0.15f};           ///< Зазор над головкой рельса (УГР) для исключения рельсов/шпал (м)
  float undercarriage_half_width_{1.15f};     ///< Полуширина подвагонного габарита до контактного рельса (м)
  float contact_rail_height_{0.50f};          ///< Верхняя отметка зоны контактного рельса над УГР (м)
  float platform_clearance_half_width_{1.33f};///< Полуширина на уровне платформы с зазором 7 см до платформы (м)
  float platform_height_{1.25f};              ///< Верхняя отметка платформы станции над УГР (м)
  float waist_half_width_{1.37f};             ///< Полуширина кузова вагона по подоконному поясу (м)
  float carriage_wall_height_{2.60f};         ///< Высота вертикальной стенки кузова до ската крыши (м)
  float roof_half_width_{0.85f};              ///< Полуширина верха крыши вагона (м)
  float carriage_height_{3.60f};              ///< Полная габаритная высота вагона от УГР (м)
  float envelope_alpha_{0.18f};               ///< Прозрачность 3D-объема габарита (0.0 - 1.0)
  float temporal_alpha_{0.30f};               ///< Коэффициент экспоненциального сглаживания EMA по кадрам

  size_t frame_count_{0};
  std::vector<TrackWaypoint> prev_trajectory_;

  // Preallocated buffer for parsing point cloud points to avoid allocations per frame
  std::vector<Point3D> points_scratch_buf_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace metro_tunnel_tracker

