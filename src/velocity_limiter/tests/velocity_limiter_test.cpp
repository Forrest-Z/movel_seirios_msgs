#include <gtest/gtest.h>
#include <velocity_limiter/velocity_limiter.h>
#include <velocity_limiter/velocity_grid.h>
#include <thread>

class VelocityLimiterTest : public ::testing::Test
{
protected:
  Set set;
  VelocityLimiter limiter;
  double p_grid_resolution_;
  bool in_grid;
  VelocityGrid velocity_grid;

  VelocityLimiterTest()
  {
    p_grid_resolution_ = 0.01;

    Zone zone_0;
    zone_0.type = "linear";
    zone_0.direction = "positive";
    zone_0.dimension = "x";
    Frontier frontier_0_1;
    for (const auto x : { -0.65, -0.65, 0.65, 2.0, 0.65, -0.65 })
    {
      frontier_0_1.shape.x_list.push_back(x);
    }
    for (const auto y : { -0.48, 0.48, 0.48, 0.0, -0.48, -0.48 })
    {
      frontier_0_1.shape.y_list.push_back(y);
    }
    frontier_0_1.inclusion = 2;
    frontier_0_1.value = 1.0;
    Frontier frontier_0_2;
    for (const auto x : { -0.65, -0.65, 0.65, 1.0, 0.65, -0.65 })
    {
      frontier_0_2.shape.x_list.push_back(x);
    }
    for (const auto y : { -0.48, 0.48, 0.48, 0.0, -0.48, -0.48 })
    {
      frontier_0_2.shape.y_list.push_back(y);
    }
    frontier_0_2.inclusion = 1;
    frontier_0_2.value = 0.5;
    Frontier frontier_0_3;
    for (const auto x : { -0.65, -0.65, 0.65, 0.65, -0.65 })
    {
      frontier_0_3.shape.x_list.push_back(x);
    }
    for (const auto y : { -0.48, 0.48, 0.48, -0.48, -0.48 })
    {
      frontier_0_3.shape.y_list.push_back(y);
    }
    frontier_0_3.inclusion = 0;
    frontier_0_3.value = 0;
    zone_0.frontier_list.push_back(frontier_0_1);
    zone_0.frontier_list.push_back(frontier_0_2);
    zone_0.frontier_list.push_back(frontier_0_3);
    set.zone_list.push_back(zone_0);

    // Zone zone_1;
    // zone_1.id = LINEAR_NEGATIVE_X;
    // Frontier frontier_1_1;
    // for(const auto x : {-0.65, -2.0, -0.65, 0.65, 0.65, -0.65})
    // {
    //   frontier_1_1.shape.x_list.push_back(x);
    // }
    // for(const auto y : {-0.48, 0.0, 0.48, 0.48, -0.48, -0.48})
    // {
    //   frontier_1_1.shape.y_list.push_back(y);
    // }
    // frontier_1_1.inclusion = 2;
    // frontier_1_1.value = 1.0;
    // Frontier frontier_1_2;
    // for(const auto x : {-0.65, -1.0, -0.65, 0.65, 0.65, -0.65})
    // {
    //   frontier_1_2.shape.x_list.push_back(x);
    // }
    // for(const auto y : {-0.48, 0.0, 0.48, 0.48, -0.48, -0.48})
    // {
    //   frontier_1_2.shape.y_list.push_back(y);
    // }
    // frontier_1_2.inclusion = 1;
    // frontier_1_2.value = 0.5;
    // zone_1.frontier_list.push_back(frontier_1_1);
    // zone_1.frontier_list.push_back(frontier_1_2);
    // set.zone_list.push_back(zone_1);
    //
    // Zone zone_2;
    // zone_2.id = ANGULAR_POSITIVE_Z;
    // Frontier frontier_2_1;
    // for(const auto x : {-0.65, -0.65, 0.0, 0.65, 0.65, -0.65})
    // {
    //   frontier_2_1.shape.x_list.push_back(x);
    // }
    // for(const auto y : {-0.48, 0.48, 2.0, 0.48, -0.48, -0.48})
    // {
    //   frontier_2_1.shape.y_list.push_back(y);
    // }
    // frontier_2_1.inclusion = 2;
    // frontier_2_1.value = 1.0;
    // Frontier frontier_2_2;
    // for(const auto x : {-0.65, -0.65, 0.0, 0.65, 0.65, -0.65})
    // {
    //   frontier_2_2.shape.x_list.push_back(x);
    // }
    // for(const auto y : {-0.48, 0.48, 1.0, 0.48, -0.48, -0.48})
    // {
    //   frontier_2_2.shape.y_list.push_back(y);
    // }
    // frontier_2_2.inclusion = 1;
    // frontier_2_2.value = 0.5;
    // zone_2.frontier_list.push_back(frontier_2_1);
    // zone_2.frontier_list.push_back(frontier_2_2);
    // set.zone_list.push_back(zone_2);
    //
    // Zone zone_3;
    // zone_3.id = ANGULAR_NEGATIVE_Z;
    // Frontier frontier_3_1;
    // for(const auto x : {-0.65, -0.65, 0.65, 0.65, 0.0, -0.65})
    // {
    //   frontier_3_1.shape.x_list.push_back(x);
    // }
    // for(const auto y : {-0.48, 0.48, 0.48, -0.48, -2.0, -0.48})
    // {
    //   frontier_3_1.shape.y_list.push_back(y);
    // }
    // frontier_3_1.inclusion = 2;
    // frontier_3_1.value = 1.0;
    // Frontier frontier_3_2;
    // for(const auto x : {-0.65, -0.65, 0.65, 0.65, 0.0, -0.65})
    // {
    //   frontier_3_2.shape.x_list.push_back(x);
    // }
    // for(const auto y : {-0.48, 0.48, 0.48, -0.48, -1.0, -0.48})
    // {
    //   frontier_3_2.shape.y_list.push_back(y);
    // }
    // frontier_3_2.inclusion = 1;
    // frontier_3_2.value = 0.5;
    // zone_3.frontier_list.push_back(frontier_3_1);
    // zone_3.frontier_list.push_back(frontier_3_2);
    // set.zone_list.push_back(zone_3);

    if (!limiter.buildLimitSet(set))
      ROS_FATAL("Fail to build limit set.");
    velocity_grid.load(set.zone_list, p_grid_resolution_);
  }
};

// Tests on calculated limit values
TEST_F(VelocityLimiterTest, limitValueOnBorderLinPos)
{
  VelocityLimit velocity_limit;

  Frontier frontier = set.zone_list[0].frontier_list[2];
  double x = 2;
  double y = 0;
  in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
  EXPECT_TRUE(in_grid);
  EXPECT_EQ(velocity_limit.linear.positive.x, frontier.value);

  // Frontier frontier = set.zone_list[0].frontier_list[0];
  // double x = 1;
  // double y = 0;
  // in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
  // EXPECT_TRUE(in_grid);
  // EXPECT_EQ(velocity_limit.linear.positive.x, frontier.value);
}

// TEST_F(VelocityLimiterTest, limitValueOnBorderLinNeg)
// {
//   VelocityLimit velocity_limit;
//
//   Frontier frontier = set.zone_list[1].frontier_list[0];
//   double x = -2;
//   double y = 0;
//   in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_EQ(velocity_limit.linear.negative.x, frontier.value);
//
//   Frontier frontier = set.zone_list[1].frontier_list[1];
//   double x = -1;
//   double y = 0;
//   in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_EQ(velocity_limit.linear.negative.x, frontier.value);
// }
//
// TEST_F(VelocityLimiterTest, limitValueOnBorderAngPos)
// {
//   VelocityLimit velocity_limit;
//
//   Frontier frontier = set.zone_list[2].frontier_list[0];
//   double x = 0;
//   double y = 2;
//   in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_EQ(velocity_limit.angular.positive.z, frontier.value);
//
//   Frontier frontier = set.zone_list[2].frontier_list[1];
//   double x = 0;
//   double y = 1;
//   in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_EQ(velocity_limit.angular.positive.z, frontier.value);
// }
//
// TEST_F(VelocityLimiterTest, limitValueOnBorderAngNeg)
// {
//   VelocityLimit velocity_limit;
//
//   Frontier frontier = set.zone_list[3].frontier_list[0];
//   double x = 0;
//   double y = -2;
//   in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_EQ(velocity_limit.angular.negative.z, frontier.value);
//
//   Frontier frontier = set.zone_list[3].frontier_list[1];
//   double x = 0;
//   double y = -1;
//   in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_EQ(velocity_limit.angular.negative.z, frontier.value);
// }

TEST_F(VelocityLimiterTest, limitValueOnCritical)
{
  VelocityLimit velocity_limit;

  Frontier frontier_inner_most = set.zone_list[0].frontier_list[0];
  double x = frontier_inner_most.shape.x_list[0];
  double y = frontier_inner_most.shape.y_list[0];
  in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
  EXPECT_TRUE(in_grid);
  EXPECT_EQ(velocity_limit.linear.positive.x, 0);
  // EXPECT_EQ(velocity_limit.linear.negative.x, 0);
  // EXPECT_EQ(velocity_limit.angular.positive.z, 0);
  // EXPECT_EQ(velocity_limit.angular.negative.z, 0);
}

TEST_F(VelocityLimiterTest, limitValueInCritical)
{
  VelocityLimit velocity_limit;

  // Frontier frontier_inner_most = set.zone_list[0].frontier_list[0];
  double x = 0;
  double y = 0;
  in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
  EXPECT_TRUE(in_grid);
  EXPECT_EQ(velocity_limit.linear.positive.x, 0);
  // EXPECT_EQ(velocity_limit.linear.negative.x, 0);
  // EXPECT_EQ(velocity_limit.angular.positive.z, 0);
  // EXPECT_EQ(velocity_limit.angular.negative.z, 0);
}

TEST_F(VelocityLimiterTest, limitValueBetweenBordersLinPos1)
{
  VelocityLimit velocity_limit;

  Frontier frontier_inner = set.zone_list[0].frontier_list[2];
  Frontier frontier_outer = set.zone_list[0].frontier_list[1];
  ROS_INFO_STREAM("Inclusion of 0: " << frontier_inner.inclusion);
  double x = 0.8;
  double y = 0;
  in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
  EXPECT_TRUE(in_grid);
  EXPECT_LT(velocity_limit.linear.positive.x, frontier_outer.value);
  EXPECT_GT(velocity_limit.linear.positive.x, 0);
  EXPECT_FLOAT_EQ(velocity_limit.linear.positive.x, 3.0 / 14.0);
}

TEST_F(VelocityLimiterTest, limitValueBetweenBordersLinPos2)
{
  VelocityLimit velocity_limit;

  Frontier frontier_inner = set.zone_list[0].frontier_list[1];
  Frontier frontier_outer = set.zone_list[0].frontier_list[2];
  double x = 1.5;
  double y = 0;
  in_grid = velocity_grid.getVelocityLimit(x, y, velocity_limit);
  EXPECT_TRUE(in_grid);
  EXPECT_LT(velocity_limit.linear.positive.x, frontier_outer.value);
  EXPECT_GT(velocity_limit.linear.positive.x, frontier_inner.value);
  EXPECT_FLOAT_EQ(velocity_limit.linear.positive.x, 0.75);
}

// TEST_F(VelocityLimiterTest, limitValueBetweenBordersLinNeg)
// {
//   VelocityLimit velocity_limit;
//
//   CriticalFrontier frontier_inner = set.critical_frontier;
//   Frontier frontier_outer = set.zone_list[1].frontier_list[0];
//   double x = -0.825;
//   double y = 0;
//   in_grid = velocity_gird.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_DOUBLE_EQ(velocity_limit.linear.negative.x, frontier_outer.value*(0.825 - 0.65)/(1 - 0.65));
//
//
//   Frontier frontier_inner = set.zone_list[1].frontier_list[0];
//   Frontier frontier_outer = set.zone_list[1].frontier_list[1];
//   double x = -1.5;
//   double y = 0;
//   in_grid = velocity_gird.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_DOUBLE_EQ(velocity_limit.linear.negative.x, frontier_inner.value + (frontier_outer.value -
//   frontier_inner.value)*(1.5 - 1)/(2-1));
//
// }
//
// TEST_F(VelocityLimiterTest, limitValueBetweenBordersAngPos)
// {
//   VelocityLimit velocity_limit;
//
//   CriticalFrontier frontier_inner = set.critical_frontier;
//   Frontier frontier_outer = set.zone_list[2].frontier_list[0];
//   double x = 0;
//   double y = 1.5;
//   in_grid = velocity_gird.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_DOUBLE_EQ(velocity_limit.angular.positive.z, frontier_outer.value*(0.825 - 0.65)/(1 - 0.65));
//
//
//   Frontier frontier_inner = set.zone_list[2].frontier_list[0];
//   Frontier frontier_outer = set.zone_list[2].frontier_list[1];
//   double x = 0;
//   double y = 0.825;
//   in_grid = velocity_gird.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_DOUBLE_EQ(velocity_limit.angular.positive.z, frontier_inner.value + (frontier_outer.value -
//   frontier_inner.value)*(1.5 - 1)/(2-1));
//
// }
//
// TEST_F(VelocityLimiterTest, limitValueBetweenBordersAngNeg)
// {
//   VelocityLimit velocity_limit;
//
//   CriticalFrontier frontier_inner = set.critical_frontier;
//   Frontier frontier_outer = set.zone_list[3].frontier_list[0];
//   double x = 0;
//   double y = -0.825;
//   in_grid = velocity_gird.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_DOUBLE_EQ(velocity_limit.angular.negative.z, frontier_outer.value*(0.825 - 0.65)/(1 - 0.65));
//
//
//   Frontier frontier_inner = set.zone_list[3].frontier_list[0];
//   Frontier frontier_outer = set.zone_list[3].frontier_list[1];
//   double x = 0;
//   double y = -1.5;
//   in_grid = velocity_gird.getVelocityLimit(x, y, velocity_limit);
//   EXPECT_TRUE(in_grid);
//   EXPECT_DOUBLE_EQ(velocity_limit.angular.negative.z, frontier_inner.value + (frontier_outer.value -
//   frontier_inner.value)*(1.5 - 1)/(2-1));
//
// }

// Tests on whether velocity is limited
TEST_F(VelocityLimiterTest, velocityLimitLin)
{
  VelocityLimit velocity_limit;
  velocity_limit.linear.positive.x = 1.0;
  geometry_msgs::Twist velocity_in;
  velocity_in.linear.x = 2.0;
  geometry_msgs::Twist velocity_limited;
  limiter.limitVelocity(velocity_in, velocity_limited, velocity_limit);

  EXPECT_LE(velocity_limited.linear.x, velocity_limit.linear.positive.x);
}

TEST_F(VelocityLimiterTest, velocityLimitLinAng)
{
  VelocityLimit velocity_limit;
  velocity_limit.linear.positive.x = 1.0;
  velocity_limit.angular.positive.z = 2.0;
  geometry_msgs::Twist velocity_in;
  velocity_in.linear.x = 2.0;
  velocity_in.angular.z = 3.0;
  geometry_msgs::Twist velocity_limited;
  limiter.limitVelocity(velocity_in, velocity_limited, velocity_limit);

  EXPECT_LE(velocity_limited.linear.x, velocity_limit.linear.positive.x);
  EXPECT_LE(velocity_limited.angular.z, velocity_limit.angular.positive.z);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
