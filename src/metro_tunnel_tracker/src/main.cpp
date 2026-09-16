#include "metro_tunnel_tracker/tunnel_tracker_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<metro_tunnel_tracker::TunnelTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
