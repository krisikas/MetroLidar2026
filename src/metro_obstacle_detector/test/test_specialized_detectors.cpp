#include <gtest/gtest.h>
#include "metro_obstacle_detector/specialized_detectors.hpp"
#include <cmath>

using namespace metro_obstacle_detector;

TEST(SpecializedDetectorsTest, InfrastructureExclusionRules)
{
  SpecializedDetectors detector;

  // 1. Track bed / ballast (v <= 0.12m)
  EXPECT_TRUE(detector.is_infrastructure(0.0f, 0.05f));
  EXPECT_TRUE(detector.is_infrastructure(0.0f, 0.12f));

  // 2. Contact rail (|u| in [1.05, 1.50]m at v in [0.05, 0.45]m)
  EXPECT_TRUE(detector.is_infrastructure(1.20f, 0.25f));
  EXPECT_TRUE(detector.is_infrastructure(-1.30f, 0.20f));

  // 3. Platform edge (|u| in [1.15, 1.85]m at v in [0.40, 1.25]m)
  EXPECT_TRUE(detector.is_infrastructure(1.40f, 0.80f));
  EXPECT_TRUE(detector.is_infrastructure(-1.50f, 1.00f));

  // 4. Legitimate obstacles: low bar on rails (u=0, v=0.20m) -> NOT infrastructure
  EXPECT_FALSE(detector.is_infrastructure(0.0f, 0.20f));

  // 5. Legitimate obstacles: suspended filament (u=0, v=2.00m) -> NOT infrastructure
  EXPECT_FALSE(detector.is_infrastructure(0.0f, 2.00f));

  // 6. Legitimate obstacles: bulk body inside car clearance (u=0.5m, v=1.5m) -> NOT infrastructure
  EXPECT_FALSE(detector.is_infrastructure(0.50f, 1.50f));
}
