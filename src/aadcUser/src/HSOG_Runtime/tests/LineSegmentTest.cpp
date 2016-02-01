#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/utils/geometry/LineSegment.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class LineSegmentTest: public ::testing::Test {
protected:
  LineSegmentTest() {};
  virtual ~LineSegmentTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};

TEST_F(LineSegmentTest, testIsOnSegmentAbove180) {
  double radius = 2;
  double length = (2 * M_PI * radius) * 0.75;
  double angle = length / radius;
  LineSegment segment = LineSegment(Pose2D(0, 0), length, angle);
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(0, 2)));
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.5, 0.1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 2)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.5, 3.9)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.5, 0.1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, 2)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, 3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1.5, 3.9)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, 5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-3, 1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.9, 1.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(0, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.9, 1.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(3, 1)));
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-3, 3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1.9, 3.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, 3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(0, 3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.9, 3.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(3, 3)));
}

TEST_F(LineSegmentTest, testIsOnSegmentAboveNeg180) {
  double radius = -2;
  double length = (2 * M_PI * radius) * -0.75;
  double angle = length / radius;
  LineSegment segment = LineSegment(Pose2D(0, 0), length, angle);
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(0, -2)));
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.5, -0.1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -2)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.5, -3.9)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.5, -0.1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, -2)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, -3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1.5, -3.9)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, -5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-3, -1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.9, -1.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(0, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.9, -1.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(3, -1)));
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-3, -3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1.9, -3.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(-1, -3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(0, -3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -3)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.9, -3.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(3, -3)));
}

TEST_F(LineSegmentTest, testIsOnSegmentBelow180) {
  double radius = 2;
  double length = (2 * M_PI * radius) * 0.25;
  double angle = length / radius;
  LineSegment segment = LineSegment(Pose2D(0, 0), length, angle);
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(0.1, 2)));
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.5, 0.1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 2)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1, 3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1.5, 3.9)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1, 5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.5, 0.1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 2)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.5, 3.9)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-3, 1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.9, 1.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(0, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.9, 1.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(3, 1)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-3, 3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.9, 3.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(0, 3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1, 3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1.9, 3.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(3, 3)));
}

TEST_F(LineSegmentTest, testIsOnSegmentBelowNeg180) {
  double radius = -2;
  double length = (2 * M_PI * radius) * -0.25;
  double angle = length / radius;
  LineSegment segment = LineSegment(Pose2D(0, 0), length, angle);
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(0.1, -2)));
  
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, 1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.5, -0.1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -2)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1, -3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1.5, -3.9)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1, -5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, 1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.5, -0.1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -2)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.5, -3.9)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -5)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-3, -1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.9, -1.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -1)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(0, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1, -1)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(1.9, -1.5)));
  ASSERT_TRUE(segment.isOnSegment(Vector2d(3, -1)));
  
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-3, -3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1.9, -3.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(-1, -3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(0, -3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1, -3)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(1.9, -3.5)));
  ASSERT_FALSE(segment.isOnSegment(Vector2d(3, -3)));
}

TEST_F(LineSegmentTest, testGetDistancePos) {
  double radius = 2;
  double length = (2 * M_PI * radius) * 0.25;
  double angle = length / radius;
  LineSegment segment = LineSegment(Pose2D(0, 0), length, angle);
  
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, 0)), 0, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(2, 0)), 0.828427, 0.0001);
  
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(1, 2)), 1, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, 1.9)), 1.9, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, 2)), 2, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0.1, 2)), 1.9, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, 1)), 1, 0.0001);
}

TEST_F(LineSegmentTest, testGetDistanceNeg) {
  double radius = -2;
  double length = (2 * M_PI * radius) * -0.25;
  double angle = length / radius;
  LineSegment segment = LineSegment(Pose2D(0, 0), length, angle);
  
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, 0)), 0, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(2, 0)), 0.828427, 0.0001);
  
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(1, -2)), 1, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, -1.9)), 1.9, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, -2)), 2, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0.1, -2)), 1.9, 0.0001);
  ASSERT_NEAR(segment.getDistanceTo(Vector2d(0, -1)), 1, 0.0001);
}
}
