#include <gtest/gtest.h>
#include "metro_tunnel_tracker/rail_geometry_tracker.hpp"
#include <cmath>

namespace metro_tunnel_tracker
{

TEST(RailGeometryTrackerTest, StraightTunnelTracking)
{
  TrackerConfig config;
  config.lookahead_distance = 50.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.5f;
  config.single_tunnel_radius = 2.2f;

  RailGeometryTracker tracker(config);

  std::vector<Point3D> points;
  for (float y = -2.5f; y >= -50.0f; y -= 0.2f) {
    // Floor
    for (float x = -1.0f; x <= 1.0f; x += 0.1f) {
      points.push_back(Point3D{x, y, -1.35f, 10.0f});
    }
    // Left wall at -2.2m
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back(Point3D{-2.2f, y, z, 20.0f});
    }
    // Right wall at +2.2m
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back(Point3D{2.2f, y, z, 20.0f});
    }
  }

  auto traj = tracker.estimate_track_trajectory(points);
  ASSERT_FALSE(traj.empty());

  for (const auto & wp : traj) {
    EXPECT_NEAR(wp.x, 0.0f, 0.10f);
    EXPECT_NEAR(wp.z_rail, -1.35f, 0.10f);
    EXPECT_TRUE(wp.valid);
  }
}

TEST(RailGeometryTrackerTest, AsymmetricWallExpansion)
{
  TrackerConfig config;
  config.lookahead_distance = 50.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.5f;
  config.single_tunnel_radius = 2.2f;

  RailGeometryTracker tracker(config);

  std::vector<Point3D> points;
  for (float y = -2.5f; y >= -50.0f; y -= 0.2f) {
    for (float x = -1.0f; x <= 1.0f; x += 0.1f) {
      points.push_back(Point3D{x, y, -1.35f, 10.0f});
    }
    // Left wall stays continuous at -2.2m
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back(Point3D{-2.2f, y, z, 20.0f});
    }
    // Right wall expands to +6.0m (platform or second track)
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back(Point3D{6.0f, y, z, 20.0f});
    }
  }

  auto traj = tracker.estimate_track_trajectory(points);
  ASSERT_FALSE(traj.empty());

  for (const auto & wp : traj) {
    // Should stay anchored to left wall + radius = -2.2 + 2.2 = 0.0m
    EXPECT_NEAR(wp.x, 0.0f, 0.20f);
    EXPECT_NEAR(wp.z_rail, -1.35f, 0.10f);
  }
}

TEST(RailGeometryTrackerTest, RailwayCurvatureConstraint)
{
  TrackerConfig config;
  config.lookahead_distance = 50.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.5f;
  config.min_curve_radius = 200.0f;

  RailGeometryTracker tracker(config);

  std::vector<Point3D> points;
  // Step discontinuity at y = -20.0m (e.g. wall suddenly steps 3 meters)
  for (float y = -2.5f; y >= -50.0f; y -= 0.2f) {
    float offset = (y < -20.0f) ? 3.0f : 0.0f;
    for (float x = -1.0f; x <= 1.0f; x += 0.1f) {
      points.push_back(Point3D{x + offset, y, -1.35f, 10.0f});
    }
    for (float z = -0.5f; z <= 1.5f; z += 0.2f) {
      points.push_back(Point3D{-2.2f + offset, y, z, 20.0f});
      points.push_back(Point3D{2.2f + offset, y, z, 20.0f});
    }
  }

  auto traj = tracker.estimate_track_trajectory(points);
  ASSERT_FALSE(traj.empty());

  for (size_t i = 1; i < traj.size(); ++i) {
    float dx = std::abs(traj[i].x - traj[i - 1].x);
    // Max displacement per slice step (2.5m) cannot exceed realistic curvature turn rate
    EXPECT_LE(dx, 0.45f);
  }
}

TEST(RailGeometryTrackerTest, SquareTunnelTracking)
{
  TrackerConfig config;
  config.lookahead_distance = 60.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.2f;
  config.single_tunnel_radius = 2.0f;

  RailGeometryTracker tracker(config);

  std::vector<Point3D> points;
  for (float y = -2.5f; y >= -60.0f; y -= 0.25f) {
    // Floor
    for (float x = -1.8f; x <= 1.8f; x += 0.2f) {
      points.push_back(Point3D{x, y, -1.35f, 10.0f});
    }
    // Flat vertical left wall at -2.0m
    for (float z = -0.5f; z <= 2.2f; z += 0.25f) {
      points.push_back(Point3D{-2.0f, y, z, 20.0f});
    }
    // Flat vertical right wall at +2.0m
    for (float z = -0.5f; z <= 2.2f; z += 0.25f) {
      points.push_back(Point3D{2.0f, y, z, 20.0f});
    }
    // Flat ceiling at +2.2m
    for (float x = -1.8f; x <= 1.8f; x += 0.3f) {
      points.push_back(Point3D{x, y, 2.2f, 15.0f});
    }
  }

  auto traj = tracker.estimate_track_trajectory(points);
  ASSERT_FALSE(traj.empty());

  for (const auto & wp : traj) {
    EXPECT_NEAR(wp.x, 0.0f, 0.10f);
    EXPECT_NEAR(wp.z_rail, -1.35f, 0.10f);
    EXPECT_TRUE(wp.valid);
  }
}

TEST(RailGeometryTrackerTest, SparseLongRangeCloud)
{
  TrackerConfig config;
  config.lookahead_distance = 180.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.2f;

  RailGeometryTracker tracker(config);

  std::vector<Point3D> points;
  // Sparse points simulating 1/R^2 drop at 100-180m
  for (float y = -2.5f; y >= -180.0f; y -= 1.5f) {
    points.push_back(Point3D{-0.76f, y, -1.35f, 10.0f}); // left rail
    points.push_back(Point3D{+0.76f, y, -1.35f, 10.0f}); // right rail
    points.push_back(Point3D{-2.15f, y, 0.5f, 20.0f});   // left wall
    points.push_back(Point3D{+2.15f, y, 0.5f, 20.0f});   // right wall
  }

  auto traj = tracker.estimate_track_trajectory(points);
  ASSERT_FALSE(traj.empty());
  EXPECT_GE(traj.back().y, -180.0f);

  for (const auto & wp : traj) {
    EXPECT_NEAR(wp.x, 0.0f, 0.15f);
    EXPECT_NEAR(wp.z_rail, -1.35f, 0.12f);
    EXPECT_TRUE(wp.valid);
  }
}

TEST(RailGeometryTrackerTest, CurveSingleWallTracking)
{
  TrackerConfig config;
  config.lookahead_distance = 60.0f;
  config.min_distance = 2.5f;
  config.slice_step = 2.2f;
  config.min_curve_radius = 160.0f;
  config.single_tunnel_radius = 2.15f;

  RailGeometryTracker tracker(config);

  const float R = 250.0f; // 250m radius curve turning to the left (negative X)
  std::vector<Point3D> points;

  for (float y = -2.5f; y >= -60.0f; y -= 0.3f) {
    const float dist = -y;
    const float x_center = -(dist * dist) / (2.0f * R);

    // Floor track bed
    for (float dx = -0.8f; dx <= 0.8f; dx += 0.2f) {
      points.push_back(Point3D{x_center + dx, y, -1.35f, 10.0f});
    }

    // Up to 20m: both walls visible
    if (dist < 20.0f) {
      for (float z = -0.5f; z <= 1.5f; z += 0.3f) {
        points.push_back(Point3D{x_center - 2.15f, y, z, 20.0f}); // left wall
        points.push_back(Point3D{x_center + 2.15f, y, z, 20.0f}); // right wall
      }
    } else {
      // Beyond 20m: ONLY outer right wall is visible with heavy density (10x points)
      // Inner left wall is completely occluded / outside FOV (0 points)
      for (float z = -0.5f; z <= 2.5f; z += 0.1f) {
        for (float sub_y = y - 0.1f; sub_y <= y + 0.1f; sub_y += 0.05f) {
          points.push_back(Point3D{x_center + 2.15f, sub_y, z, 30.0f});
        }
      }
    }
  }

  auto traj = tracker.estimate_track_trajectory(points);
  ASSERT_FALSE(traj.empty());

  for (const auto & wp : traj) {
    const float dist = -wp.y;
    const float expected_x = -(dist * dist) / (2.0f * R);
    // Trajectory must follow the true curve accurately without straightening out
    // and must NOT be pulled towards the dense right wall
    EXPECT_NEAR(wp.x, expected_x, 0.35f);
    EXPECT_TRUE(wp.valid);
  }
}

}  // namespace metro_tunnel_tracker
