#pragma once

#include <vector>

namespace metro_tunnel_tracker
{

struct Point3D
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
};

struct TrackWaypoint
{
  float x{0.0f};
  float y{0.0f};
  float z_rail{0.0f};
  float yaw{0.0f};
  float pitch{0.0f};
  float left_boundary{-2.2f};
  float right_boundary{2.2f};
  bool valid{false};
};

}  // namespace metro_tunnel_tracker

