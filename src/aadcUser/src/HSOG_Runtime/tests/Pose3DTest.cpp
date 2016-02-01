#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Pose3D.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class Pose3DTest: public ::testing::Test {
protected:
  Pose3DTest() {};
  virtual ~Pose3DTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {}
};

TEST_F(Pose3DTest, testApplyTo) {
  Pose3D testee(Vector3d(3, 1, 0), AngleAxisd(M_PI / 2, Vector3d::UnitZ()));

  // Test applyTo other pose
  Pose3D otherPose(Vector3d(1, 0, 0), AngleAxisd((20 * M_PI / 180), Vector3d::UnitZ()));
  Pose3D resPose = testee * otherPose;

  Quaterniond expectedOrientation(AngleAxisd((110 * M_PI / 180), Vector3d::UnitZ()));
  ASSERT_NEAR(3, resPose.x(), 0.0001);
  ASSERT_NEAR(2, resPose.y(), 0.0001);
  ASSERT_NEAR(0, expectedOrientation.angularDistance(resPose.getOrientation()), 0.0001);
  
  // Test applyInverseTo other pose
  Pose3D resPoseInverse = testee / resPose;

  ASSERT_NEAR(otherPose.x(), resPoseInverse.x(), 0.0001);
  ASSERT_NEAR(otherPose.y(), resPoseInverse.y(), 0.0001);
  ASSERT_NEAR(0, otherPose.getOrientation().angularDistance(resPoseInverse.getOrientation()), 0.0001);

  // Test applyTo other position
  Vector3d otherPosition(2, 1, 0);
  Vector3d resPos = testee * otherPosition;

  ASSERT_NEAR(2, resPos(0), 0.0001);
  ASSERT_NEAR(3, resPos(1), 0.0001);

  // Test applyInverseTo other position
  Vector3d resPosInverse = testee / resPos;

  ASSERT_NEAR(otherPosition(0), resPosInverse(0), 0.0001);
  ASSERT_NEAR(otherPosition(1), resPosInverse(1), 0.0001);
}

TEST_F(Pose3DTest, testApplyInverseTo) {
  Pose3D testee(Vector3d(3, 1, 0), AngleAxisd(M_PI / 2, Vector3d::UnitZ()));

  // Test applyTo other pose
  Pose3D otherPose(Vector3d(1, 0, 0), AngleAxisd(20 * M_PI / 180, Vector3d::UnitZ()));
  Pose3D resPose = testee / otherPose;

  Quaterniond expectedOrientation(AngleAxisd(-70 * M_PI / 180, Vector3d::UnitZ()));
  ASSERT_NEAR(-1, resPose.x(), 0.0001);
  ASSERT_NEAR(2, resPose.y(), 0.0001);
  ASSERT_NEAR(0, expectedOrientation.angularDistance(resPose.getOrientation()), 0.0001);

  // Test applyTo other position
  Vector3d otherPosition(2, 1, 0);
  Vector3d resPos = testee / otherPosition;

  ASSERT_NEAR(0, resPos(0), 0.0001);
  ASSERT_NEAR(1, resPos(1), 0.0001);
}

}
