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

}  // namespace metro_tunnel_tracker
