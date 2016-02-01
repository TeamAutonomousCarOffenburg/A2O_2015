#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/utils/geometry/Pose2D.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class Pose2DTest: public ::testing::Test {
protected:
  Pose2DTest() {};
  virtual ~Pose2DTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};

TEST_F(Pose2DTest, testApplyTo) {
  Pose2D testee(3, 1, Angle::deg(90));

  // Test applyTo other pose
  Pose2D otherPose(1, 0, Angle::deg(20));
  Pose2D resPose(testee * otherPose);

  ASSERT_NEAR(3, resPose.x(), 0.000001);
  ASSERT_NEAR(2, resPose.y(), 0.000001);
  ASSERT_NEAR(110, resPose.getAngle().deg(), 0.000001);

  // Test applyInverseTo other pose
  Pose2D resPoseInverse = testee / resPose;

  ASSERT_NEAR(otherPose.x(), resPoseInverse.x(), 0.000001);
  ASSERT_NEAR(otherPose.y(), resPoseInverse.y(), 0.000001);
  ASSERT_NEAR(otherPose.getAngle().deg(), resPoseInverse.getAngle().deg(), 0.000001);

  // Test applyTo other position
  Vector2d otherPosition(2, 1);
  Vector2d resPos = testee * otherPosition;

  ASSERT_NEAR(2, resPos(0), 0.000001);
  ASSERT_NEAR(3, resPos(1), 0.000001);

  // Test applyInverseTo other position
  Vector2d resPosInverse = testee / resPos;

  ASSERT_NEAR(otherPosition(0), resPosInverse(0), 0.000001);
  ASSERT_NEAR(otherPosition(1), resPosInverse(1), 0.000001);
}

TEST_F(Pose2DTest, testApplyInverseTo) {
  Pose2D testee(3, 1, Angle::deg(90));

  // Test applyTo other pose
  Pose2D otherPose(1, 0, Angle::deg(20));
  Pose2D resPose = testee / otherPose;

  ASSERT_NEAR(-1, resPose.x(), 0.000001);
  ASSERT_NEAR(2, resPose.y(), 0.000001);
  ASSERT_NEAR(-70, resPose.getAngle().deg(), 0.000001);

  // Test applyTo other position
  Vector2d otherPosition(2, 1);
  Vector2d resPos = testee / otherPosition;

  ASSERT_NEAR(0, resPos(0), 0.000001);
  ASSERT_NEAR(1, resPos(1), 0.000001);
}

TEST_F(Pose2DTest, getDistanceTo) {
  Pose2D pose1(-2, -2);
  Pose2D pose2(1, 2);
  ASSERT_NEAR(5.0, pose1.getDistanceTo(pose2), 0.00001);

  pose1 = Pose2D(2, 2);
  pose2 = Pose2D(-1, -2);
  ASSERT_NEAR(5.0, pose1.getDistanceTo(pose2), 0.00001);
}

TEST_F(Pose2DTest, getDeltaAngle) {
  Pose2D pose1(-2, -2, Angle::deg(90));
  Pose2D pose2(-2, -2, Angle::deg(45));
  ASSERT_NEAR(-45.0, pose1.getDeltaAngle(pose2).deg(), 0.00001);
}

TEST_F(Pose2DTest, getGetUnitVector) {
  Pose2D pose(17, 95, Angle::deg(45));
  Vector2d result = pose.getUnitVector();
  ASSERT_NEAR(result(0), 0.7071067, 0.00001);
  ASSERT_NEAR(result(1), 0.7071067, 0.00001);
}

TEST_F(Pose2DTest, getAngleTo) {
  Pose2D pose1(-2, -2, Angle::deg(90));
  Pose2D pose2(2, 2);
  ASSERT_NEAR(-45, pose1.getAngleTo(pose2).deg(), 0.00001);

  pose1 = Pose2D(2, 2, Angle::deg(91));
  pose2 = Pose2D(2, 0);
  ASSERT_NEAR(179, pose1.getAngleTo(pose2).deg(), 0.00001);
}
}
