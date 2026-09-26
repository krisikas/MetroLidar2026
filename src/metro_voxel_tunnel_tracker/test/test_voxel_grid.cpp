#include <gtest/gtest.h>
#include "metro_voxel_tunnel_tracker/voxel_grid.hpp"
#include <cmath>

namespace metro_voxel_tunnel_tracker
{

TEST(VoxelGridTest, InsertionAndCentroidCalculation)
{
  VoxelGridConfig config;
  config.voxel_size_x = 0.10f;
  config.voxel_size_y = 0.20f;
  config.voxel_size_z = 0.05f;

  SparseVoxelGrid grid(config);

  std::vector<Point3D> points = {
    {0.01f, -10.05f, -1.30f, 10.0f},
    {0.05f, -10.08f, -1.32f, 20.0f},
    {0.09f, -10.02f, -1.28f, 30.0f}
  };

  grid.insert_points(points);
  EXPECT_EQ(grid.size(), 1u);

  const auto & voxels = grid.get_voxels();
  ASSERT_EQ(voxels.size(), 1u);
  EXPECT_EQ(voxels[0].point_count, 3);
  EXPECT_NEAR(voxels[0].x, 0.05f, 1e-4f);
  EXPECT_NEAR(voxels[0].y, -10.05f, 1e-2f);
  EXPECT_NEAR(voxels[0].z, -1.30f, 1e-3f);
  EXPECT_NEAR(voxels[0].mean_intensity, 20.0f, 1e-3f);
}

TEST(VoxelGridTest, FrenetSliceQuery)
{
  VoxelGridConfig config;
  config.voxel_size_x = 0.10f;
  config.voxel_size_y = 0.20f;
  config.voxel_size_z = 0.05f;

  SparseVoxelGrid grid(config);

  std::vector<Point3D> points;
  // Create a line of points along y from -5 to -15
  for (float y = -5.0f; y >= -15.0f; y -= 0.1f) {
    points.push_back({0.0f, y, -1.35f, 10.0f});
  }
  grid.insert_points(points);

  std::vector<const Voxel *> slice_voxels;
  // Query at y = -10.0, half thickness 1.0 (spanning -9.0 to -11.0)
  grid.query_frenet_slice(0.0f, -10.0f, 0.0f, 1.0f, 2.0f, slice_voxels);

  EXPECT_FALSE(slice_voxels.empty());
  for (const auto * v : slice_voxels) {
    EXPECT_LE(v->y, -9.0f + 0.1f);
    EXPECT_GE(v->y, -11.0f - 0.1f);
  }
}

}  // namespace metro_voxel_tunnel_tracker
