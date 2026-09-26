#include <gtest/gtest.h>
#include "metro_voxel_tunnel_tracker/voxel_tunnel_tracker.hpp"
#include <cmath>

namespace metro_voxel_tunnel_tracker
{

TEST(VoxelTunnelTrackerTest, StraightTunnelTracking)
{
  VoxelTrackerConfig config;
  config.lookahead_distance = 50.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.5f;
  config.single_tunnel_radius = 2.2f;

  VoxelTunnelTracker tracker(config);

  std::vector<Point3D> points;
  for (float y = -2.5f; y >= -50.0f; y -= 0.15f) {
    // Track bed
    for (float x = -0.9f; x <= 0.9f; x += 0.1f) {
      points.push_back({x, y, -1.35f, 10.0f});
    }
    // Left wall at -2.2m
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back({-2.2f, y, z, 20.0f});
    }
    // Right wall at +2.2m
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back({2.2f, y, z, 20.0f});
    }
  }

  std::vector<TrackWaypoint> traj;
  std::vector<ObstacleCluster> obstacles;
  tracker.process(points, traj, obstacles);

  ASSERT_FALSE(traj.empty());
  for (const auto & wp : traj) {
    EXPECT_NEAR(wp.x, 0.0f, 0.10f);
    EXPECT_NEAR(wp.z_rail, -1.35f, 0.10f);
    EXPECT_TRUE(wp.valid);
  }
  EXPECT_TRUE(obstacles.empty());
}

TEST(VoxelTunnelTrackerTest, ObstacleDetectionInClearanceEnvelope)
{
  VoxelTrackerConfig config;
  config.lookahead_distance = 50.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.5f;

  VoxelTunnelTracker tracker(config);

  std::vector<Point3D> points;
  // Create straight tunnel
  for (float y = -2.5f; y >= -50.0f; y -= 0.15f) {
    for (float x = -0.9f; x <= 0.9f; x += 0.1f) {
      points.push_back({x, y, -1.35f, 10.0f});
    }
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back({-2.2f, y, z, 20.0f});
      points.push_back({2.2f, y, z, 20.0f});
    }
  }

  // Inject an obstacle (box at y = -20.0m, x = 0.0m, z = -0.5m)
  // Height above rail is -0.5 - (-1.35) = 0.85m (inside car body envelope)
  for (float oy = -20.5f; oy <= -19.5f; oy += 0.1f) {
    for (float ox = -0.3f; ox <= 0.3f; ox += 0.1f) {
      for (float oz = -0.8f; oz <= -0.2f; oz += 0.1f) {
        points.push_back({ox, oy, oz, 50.0f});
      }
    }
  }

  std::vector<TrackWaypoint> traj;
  std::vector<ObstacleCluster> obstacles;
  tracker.process(points, traj, obstacles);

  ASSERT_FALSE(traj.empty());
  ASSERT_FALSE(obstacles.empty());

  const auto & obs = obstacles.front();
  EXPECT_NEAR(obs.center_x, 0.0f, 0.25f);
  EXPECT_NEAR(obs.center_y, -20.0f, 0.50f);
  EXPECT_GT(obs.total_points, 10u);
}

}  // namespace metro_voxel_tunnel_tracker
