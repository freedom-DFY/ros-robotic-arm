#include "five_dof_arm/kinematics.hpp"
#include <gtest/gtest.h>
#include <random>

using namespace five_dof_arm;
const Positions lower{0, 0, 0, 0, 0}, upper{4095, 4095, 4095, 4095, 4095};

TEST(Kinematics, ConversionUsesFractionalTicks)
{
  EXPECT_NEAR(ticks_to_radians(2056), pi / 256, 1e-12);
  EXPECT_EQ(radians_to_ticks(ticks_to_radians(1910)), 1910);
}

TEST(Kinematics, KnownStraightArmGeometry)
{
  Kinematics kin;
  Angles q;
  q << -pi / 2, 1.444, 1.444, -pi / 2, 0;
  const auto t = kin.forward(q);
  EXPECT_NEAR(t(0, 3), kin.a2 + kin.a3, 1e-9);
  EXPECT_NEAR(t(1, 3), 0, 1e-9);
  EXPECT_NEAR(t(2, 3), -kin.tool, 1e-9);
}

TEST(Kinematics, PresetsAndRandomPosesRoundTrip)
{
  Kinematics kin;
  std::vector<Positions> poses{
    {2048, 2160, 1910, 1360, 2048}, {2048, 2048, 2048, 2048, 2048},
    {2048, 2240, 1480, 2910, 2048}};
  std::mt19937 random(42);
  std::uniform_int_distribution<int> ticks(100, 3995);
  for (int n = 0; n < 100; ++n) {
    Positions p;
    for (auto & v : p) {v = ticks(random);}
    poses.push_back(p);
  }
  for (const auto & p : poses) {
    const auto t = kin.forward(p);
    const auto result = kin.inverse(t, p, lower, upper);
    ASSERT_TRUE(result.has_value());
    for (size_t i = 0; i < 5; ++i) {EXPECT_EQ((*result)[i], p[i]);}
  }
}

TEST(Kinematics, RejectsUnreachableNonfiniteAndLimitedTargets)
{
  Kinematics kin;
  const Positions p{2048, 2160, 1910, 1360, 2048};
  Eigen::Matrix4d t = kin.forward(p);
  t(0, 3) = 10000;
  EXPECT_FALSE(kin.inverse(t, p, lower, upper));
  t(0, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kin.inverse(t, p, lower, upper));
  const Positions tight_lower{2048, 2048, 2048, 2048, 2048};
  EXPECT_FALSE(kin.inverse(kin.forward(p), p, tight_lower, tight_lower));
}

TEST(Kinematics, SolvesChangedCartesianTarget)
{
  Kinematics kin;
  const Positions p{2048, 2160, 1910, 1360, 2048};
  Eigen::Matrix4d target = kin.forward(p);
  // At base yaw pi/2, changing Y stays in the arm plane while retaining orientation.
  target(1, 3) += 10;
  const auto result = kin.inverse(target, p, lower, upper);
  ASSERT_TRUE(result.has_value());
  const auto achieved = kin.forward(*result);
  EXPECT_LT((achieved.block<3, 1>(0, 3) - target.block<3, 1>(0, 3)).norm(), 1.0);
  EXPECT_LT((achieved.block<3, 3>(0, 0) - target.block<3, 3>(0, 0)).norm(), 0.01);
}

TEST(Kinematics, CartesianInputCanChangeBaseYaw)
{
  Kinematics kin;
  const Positions p{2048, 2160, 1910, 1360, 2048};
  const Eigen::Matrix4d current = kin.forward(p);
  for (const Eigen::Vector3d & step : {Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(0, 0, 10)}) {
    const auto target = kin.position_target(p, current.block<3, 1>(0, 3) + step);
    const auto result = kin.inverse(target, p, lower, upper);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT((kin.forward(*result).block<3, 1>(0, 3) - target.block<3, 1>(0, 3)).norm(), 1.0);
  }
}
