#include "metro_obstacle_detector/cloud_parser.hpp"
#include <cstring>
#include <cmath>

namespace metro_obstacle_detector
{

CloudParser::CloudParser(const CloudParserConfig & config)
: config_(config)
{
}

void CloudParser::set_config(const CloudParserConfig & config)
{
  config_ = config;
}

bool CloudParser::parse(
  const sensor_msgs::msg::PointCloud2 & msg,
  std::vector<Point3D> & out_points,
  size_t & out_raw_count)
{
  out_points.clear();
  out_raw_count = msg.width * msg.height;

  if (out_raw_count == 0 || msg.data.empty()) {
    return false;
  }

  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_intensity = -1;
  int offset_ring = -1;
  int offset_timestamp = -1;

  for (const auto & f : msg.fields) {
    if (f.name == "x") offset_x = static_cast<int>(f.offset);
    else if (f.name == "y") offset_y = static_cast<int>(f.offset);
    else if (f.name == "z") offset_z = static_cast<int>(f.offset);
    else if (f.name == "intensity") offset_intensity = static_cast<int>(f.offset);
    else if (f.name == "ring") offset_ring = static_cast<int>(f.offset);
    else if (f.name == "timestamp") offset_timestamp = static_cast<int>(f.offset);
  }

  if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
    return false;
  }

  const size_t total_points = out_raw_count;
  const size_t point_step = msg.point_step;
  const uint8_t * raw_bytes = msg.data.data();
  const size_t data_len = msg.data.size();

  out_points.reserve(total_points);

  const float min_r_sq = config_.min_range * config_.min_range;
  const float max_range_y = -config_.max_range;
  const float max_lat = config_.max_lateral;
  const float min_z = config_.min_z;
  const float max_z = config_.max_z;

  for (size_t i = 0; i < total_points; ++i) {
    const size_t pos = i * point_step;
    if (pos + point_step > data_len) break;

    const uint8_t * ptr = raw_bytes + pos;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    std::memcpy(&x, ptr + offset_x, sizeof(float));
    std::memcpy(&y, ptr + offset_y, sizeof(float));
    std::memcpy(&z, ptr + offset_z, sizeof(float));

    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
    if (std::isinf(x) || std::isinf(y) || std::isinf(z)) continue;

    // Suppress zero-coordinate laser artifacts
    if (config_.filter_zero_points && (x == 0.0f && y == 0.0f && z == 0.0f)) continue;

    const float r_sq = x * x + y * y + z * z;
    if (r_sq < min_r_sq) continue;

    // Filter forward ROI (train moves toward -Y)
    if (y > 0.5f || y < max_range_y) continue;
    if (std::abs(x) > max_lat) continue;
    if (z < min_z || z > max_z) continue;

    float intensity = 0.0f;
    if (offset_intensity >= 0 && pos + offset_intensity + sizeof(float) <= data_len) {
      std::memcpy(&intensity, ptr + offset_intensity, sizeof(float));
    }

    uint16_t ring = 0;
    if (offset_ring >= 0 && pos + offset_ring + sizeof(uint16_t) <= data_len) {
      std::memcpy(&ring, ptr + offset_ring, sizeof(uint16_t));
    }

    double ts = 0.0;
    if (offset_timestamp >= 0 && pos + offset_timestamp + sizeof(double) <= data_len) {
      std::memcpy(&ts, ptr + offset_timestamp, sizeof(double));
    }

    out_points.push_back(Point3D{x, y, z, intensity, ring, ts});
  }

  return !out_points.empty();
}

} // namespace metro_obstacle_detector
