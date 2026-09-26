#include "metro_voxel_tunnel_tracker/voxel_tunnel_tracker_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<metro_voxel_tunnel_tracker::VoxelTunnelTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
