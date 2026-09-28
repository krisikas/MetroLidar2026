#pragma once

#include "metro_obstacle_detector/types.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <string>

namespace metro_obstacle_detector
{

struct CloudParserConfig
{
  float min_range{0.5f};            // Minimum valid range in meters (blind zone filter)
  float max_range{300.0f};          // Maximum lookahead range in meters
  float max_lateral{12.0f};         // Maximum lateral span |X| in meters
  float min_z{-4.0f};               // Vertical bounds
  float max_z{4.0f};
  bool filter_zero_points{true};    // Suppress (0,0,0) inactive beams
};

class CloudParser
{
public:
  explicit CloudParser(const CloudParserConfig & config = CloudParserConfig{});

  void set_config(const CloudParserConfig & config);
  const CloudParserConfig & get_config() const { return config_; }

  // Parse PointCloud2 message with zero-copy safety and dynamic stride/offset discovery
  bool parse(
    const sensor_msgs::msg::PointCloud2 & msg,
    std::vector<Point3D> & out_points,
    size_t & out_raw_count);

private:
  CloudParserConfig config_;
};

} // namespace metro_obstacle_detector
