#include <gtest/gtest.h>
#include "metro_obstacle_detector/clearance_envelope_sdf.hpp"
#include <cmath>

using namespace metro_obstacle_detector;

TEST(ClearanceEnvelopeSDFTest, CenterPointIsInsideNegativeSDF)
{
  ClearanceEnvelopeSDF sdf;
  // Point at track center u=0, elevation v=1.0m (platform level)
  const float d = sdf.compute_signed_distance(0.0f, 1.0f);
  EXPECT_LT(d, 0.0f);
  EXPECT_TRUE(sdf.is_inside_envelope(0.0f, 1.0f));
}

TEST(ClearanceEnvelopeSDFTest, BoundaryDistanceIsZero)
{
  ClearanceEnvelopeSDF sdf;
  // Exactly on waist boundary: u = 1.35m, v = 2.0m
  const float d_waist = sdf.compute_signed_distance(1.35f, 2.0f);
  EXPECT_NEAR(d_waist, 0.0f, 1e-4f);

  // Exactly on undercarriage boundary: u = 1.05m, v = 0.30m
  const float d_under = sdf.compute_signed_distance(1.05f, 0.30f);
  EXPECT_NEAR(d_under, 0.0f, 1e-4f);

  // Exactly on roof top corner: u = 1.10m, v = 2.80m
  const float d_roof = sdf.compute_signed_distance(1.10f, 2.80f);
  EXPECT_NEAR(d_roof, 0.0f, 1e-4f);
}

TEST(ClearanceEnvelopeSDFTest, OutsidePointsHavePositiveExactEuclideanDistance)
{
  ClearanceEnvelopeSDF sdf;
  // 10 cm outside waist boundary horizontally: u = 1.45m, v = 2.0m
  const float d_outside_waist = sdf.compute_signed_distance(1.45f, 2.0f);
  EXPECT_NEAR(d_outside_waist, 0.10f, 1e-3f);
  EXPECT_FALSE(sdf.is_inside_envelope(1.45f, 2.0f));
  EXPECT_TRUE(sdf.is_near_gauge_buffer(1.45f, 2.0f)); // Within 0.30m buffer

  // 20 cm above roof top: u = 0.0m, v = 3.00m
  const float d_above_roof = sdf.compute_signed_distance(0.0f, 3.00f);
  EXPECT_NEAR(d_above_roof, 0.20f, 1e-3f);

  // Below rail crown margin (v = 0.0m, margin = 0.12m)
  const float d_below_crown = sdf.compute_signed_distance(0.0f, 0.0f);
  EXPECT_NEAR(d_below_crown, 0.12f, 1e-3f);
}

TEST(ClearanceEnvelopeSDFTest, AllowedHalfWidthMatchesGOST9238)
{
  ClearanceEnvelopeSDF sdf;
  EXPECT_FLOAT_EQ(sdf.get_allowed_half_width(0.05f), 0.0f);  // Below 0.12m
  EXPECT_FLOAT_EQ(sdf.get_allowed_half_width(0.30f), 1.05f); // Undercarriage
  EXPECT_FLOAT_EQ(sdf.get_allowed_half_width(1.00f), 1.30f); // Platform level
  EXPECT_FLOAT_EQ(sdf.get_allowed_half_width(2.00f), 1.35f); // Waist
  EXPECT_FLOAT_EQ(sdf.get_allowed_half_width(2.80f), 1.10f); // Roof top
  EXPECT_FLOAT_EQ(sdf.get_allowed_half_width(3.00f), 0.0f);  // Above roof
}

TEST(ClearanceEnvelopeSDFTest, PolygonContourHas15PointsClosed)
{
  ClearanceEnvelopeSDF sdf;
  const auto contour = sdf.get_polygon_contour();
  EXPECT_EQ(contour.size(), 15u);
  EXPECT_FLOAT_EQ(contour.front().first, contour.back().first);
  EXPECT_FLOAT_EQ(contour.front().second, contour.back().second);
}
