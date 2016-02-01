#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/utils/geometry/Pose2D.h"
#include "../a2o/utils/geometry/Cosine.h"
#include "../a2o/utils/geometry/drive/DriveGeometry.h"
#include "../a2o/utils/geometry/drive/Driveable.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class DriveGeometryTest: public ::testing::Test {
protected:
  DriveGeometryTest() {};
  virtual ~DriveGeometryTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};



// TEST_F(DriveGeometryTest, Driveable) {
//   Pose2D carPose(0,0,Angle::Zero());
//   Pose2D nextWayPoint(2,1,Angle::Zero());
//   Driveable driveable = DriveGeometry::getCircleBeforeLineGeometry(nextWayPoint, carPose);
//   std::cout << "TEST !!! " << std::endl;
//   std::vector<Vector2d> trail = driveable.getTrail(carPose.getPosition(), nextWayPoint.getPosition());
// }
// 
// TEST_F(DriveGeometryTest, getCircleBeforeLine) {
//   Pose2D carPose(0,0,Angle::Zero());
//   Pose2D nextWayPoint(1,2,Angle::Deg_90());
//   Driveable driveable = DriveGeometry::getCircleBeforeLineGeometry(nextWayPoint, carPose);
//   std::vector<Vector2d> trail = driveable.getTrail(carPose.getPosition(), nextWayPoint.getPosition());
// }
// 
// TEST_F(DriveGeometryTest, getSCurveBeforeLine) {
//   Pose2D carPose(0,0,Angle::Zero());
//   Pose2D nextWayPoint(2,2,Angle::deg(45));
//   Driveable driveable = DriveGeometry::getSCurveBeforeLineGeometry(nextWayPoint, carPose);
//   std::vector<Vector2d> trail = driveable.getTrail(carPose.getPosition(), nextWayPoint.getPosition());
// }

/*

TEST_F(DriveGeometryTest, normalize) {
  Pose2D pose1(0, 0, Angle());
  Pose2D pose2(3, 1, Angle::deg(45));
  Cosine cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  Pose2D pose = cosine.getClosestPose(pose2.getPosition());

//   cout << "origin: x: " << cosine.origin().getPosition()(0) << " y: " << cosine.origin().getPosition()(1) << " closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  cosine.normalizeRatioToPoint(pose2.getPosition());
  pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "origin: x: " << cosine.origin().getPosition()(0) << " y: " << cosine.origin().getPosition()(1) << " closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().deg(), pose2.getAngle().deg(), 0.001);
  ASSERT_NEAR(pose.x(), pose2.x(), 0.0001);
  ASSERT_NEAR(pose.y(), pose2.y(), 0.0001);
  
  pose1 = Pose2D(0, 0, Angle());
  pose2 = Pose2D(3, 2, Angle::deg(44));
  cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  cosine.normalizeRatioToPoint(pose2.getPosition());
  pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "normalized closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().deg(), pose2.getAngle().deg(), 0.001);
  ASSERT_NEAR(pose.x(), pose2.x(), 0.0001);
  ASSERT_NEAR(pose.y(), pose2.y(), 0.0001);
  
  pose1 = Pose2D(0, 0, Angle());
  pose2 = Pose2D(3, 1, Angle::deg(20));
  cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  cosine.normalizeRatioToPoint(pose2.getPosition());
  pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "20 grad closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().deg(), pose2.getAngle().deg(), 0.001);
  ASSERT_NEAR(pose.x(), pose2.x(), 0.0001);
  ASSERT_NEAR(pose.y(), pose2.y(), 0.0001);
  
  pose1 = Pose2D(0, 0, Angle());
  pose2 = Pose2D(3, 2, Angle::deg(10));
  cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
//   pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "origin: x: " << cosine.origin().getPosition()(0) << " y: " << cosine.origin().getPosition()(1) << " closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  cosine.normalizeRatioToPoint(pose2.getPosition());
  pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "origin: x: " << cosine.origin().getPosition()(0) << " y: " << cosine.origin().getPosition()(1) << " closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().deg(), pose2.getAngle().deg(), 0.001);
  ASSERT_NEAR(pose.x(), pose2.x(), 0.0001);
  ASSERT_NEAR(pose.y(), pose2.y(), 0.0001);
  
  pose1 = Pose2D(0, 1, Angle());
  pose2 = Pose2D(M_PI, -1, Angle::deg(0));
  cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  cosine.normalizeRatioToPoint(pose2.getPosition());
  pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().deg(), pose2.getAngle().deg(), 0.001);
  ASSERT_NEAR(pose.x(), pose2.x(), 0.0001);
  ASSERT_NEAR(pose.y(), pose2.y(), 0.0001);
  
  pose1 = Pose2D(0, 1, Angle());
  pose2 = Pose2D(M_PI, -1, Angle::deg(0));
  cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  cosine.normalizeRatioToPoint(pose2.getPosition());
  pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().deg(), pose2.getAngle().deg(), 0.001);
  ASSERT_NEAR(pose.x(), pose2.x(), 0.0001);
  ASSERT_NEAR(pose.y(), pose2.y(), 0.0001);
  
  pose1 = Pose2D(0, 1, Angle());
  pose2 = Pose2D(M_PI, -1, Angle::deg(0));
  cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  cosine.normalizeRatioToPoint(pose2.getPosition());
  pose = cosine.getClosestPose(pose2.getPosition());
//   cout << "cosine closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().deg(), pose2.getAngle().deg(), 0.001);
  ASSERT_NEAR(pose.x(), pose2.x(), 0.0001);
  ASSERT_NEAR(pose.y(), pose2.y(), 0.0001);
}

TEST_F(DriveGeometryTest, cosine) {
  Pose2D pose1 = Pose2D(0, 1, Angle(-M_PI));
  Pose2D pose2 = Pose2D(M_PI, -1, Angle(-M_PI));
  Cosine cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  Pose2D pose = cosine.getClosestPose(Vector2d(0,0));
//   cout << "cosine closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().rad(), pose1.getAngle().rad(), 0.0001);
  ASSERT_NEAR(pose.y(), pose1.y(), 0.0001);
  ASSERT_NEAR(pose.x(), pose1.x(), 0.0001);
  
  pose1 = Pose2D(0, -1, Angle());
  pose2 = Pose2D(M_PI, 1, Angle::deg(0));
  cosine = DriveGeometry::getConnectingCosine(pose1, pose2);
  pose = cosine.getClosestPose(Vector2d(0,0));
//   cout << "cosine closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << " elongation: "<< cosine.elongation() << " spreading: " << cosine.spreading() <<   endl;
  ASSERT_NEAR(pose.getAngle().rad(), pose1.getAngle().rad(), 0.0001);
  ASSERT_NEAR(pose.y(), pose1.y(), 0.0001);
  ASSERT_NEAR(pose.x(), pose1.x(), 0.0001);
}

TEST_F(DriveGeometryTest, getSCurveBeforeLine) {
  Pose2D pose1 = Pose2D(0, -1, Angle(M_PI));
  Pose2D pose2 = Pose2D(M_PI, 1, Angle(M_PI));
  Driveable testee = DriveGeometry::getSCurveBeforeLineGeometry(pose1, pose2);
//   cout << "cosine: " << testee.origin().getPosition() << " angle: " << testee.origin().getAngle().deg() << " elongation: " << testee.elongation() << " spreading: " << testee.spreading() << endl;
  ASSERT_NEAR(testee.origin().getAngle().deg(), -180, 0.0001);
  ASSERT_NEAR(testee.origin().getPosition()(0), 0, 0.0001);
  ASSERT_NEAR(testee.origin().getPosition()(1), 0, 0.0001);
  ASSERT_NEAR(testee.elongation(), 1, 0.0001);
  ASSERT_NEAR(testee.spreading(), 1, 0.0001);
  
  pose1.setAngle(Angle());
  pose2.setAngle(Angle());
  testee = DriveGeometry::getSCurveBeforeLineGeometry(pose2, pose1);
//   cout << "cosine: " << testee.origin().getPosition() << " angle: " << testee.origin().getAngle().deg() << " elongation: " << testee.elongation() << " spreading: " << testee.spreading() << endl;
  ASSERT_NEAR(testee.origin().getAngle().deg(), 0, 0.0001);
  ASSERT_NEAR(testee.origin().getPosition()(0), M_PI, 0.0001);
  ASSERT_NEAR(testee.origin().getPosition()(1), 0, 0.0001);
  ASSERT_NEAR(testee.elongation(), 1, 0.0001);
  ASSERT_NEAR(testee.spreading(), 1, 0.0001);
    
  
  pose2.setPosition(5,1);
  testee = DriveGeometry::getSCurveBeforeLineGeometry(pose2, pose1);
//   cout << "cosine: " << testee.origin().getPosition() << " angle: " << testee.origin().getAngle().deg() << " elongation: " << testee.elongation() << " spreading: " << testee.spreading() << endl;
  ASSERT_NEAR(testee.origin().getAngle().deg(), 0, 0.0001);
  ASSERT_NEAR(testee.origin().getPosition()(0), M_PI, 0.0001);
  ASSERT_NEAR(testee.origin().getPosition()(1), 0, 0.0001);
  ASSERT_NEAR(testee.elongation(), 1, 0.0001);
  ASSERT_NEAR(testee.spreading(), 1, 0.0001);
  
//   pose1.setPosition(1,0);
//   pose1.setAngle(Angle());
//   pose2.setPosition(5,-1);
//   pose2.setAngle(Angle());
//   testee = DriveGeometry::getSCurveBeforeLineGeometry(pose2, pose1);
//   cout << " is valid: " << testee.isValid() << endl;
//   vector<Vector2d> points = testee.getTrail(pose1.getPosition(), pose2.getPosition());
//   cout << "cosine: " << testee.origin().getPosition() << " angle: " << testee.origin().getAngle().deg() << " elongation: " << testee.elongation() << " spreading: " << testee.spreading() << endl;
//   ASSERT_NEAR(testee.origin().getAngle().deg(), 0, 0.0001);
//   ASSERT_NEAR(testee.origin().getPosition()(0), 0, 0.0001);
//   ASSERT_NEAR(testee.origin().getPosition()(1), 0 , 0.0001);
//   ASSERT_NEAR(testee.elongation(), 2, 0.0001);
//   ASSERT_NEAR(testee.spreading(), -0.5, 0.0001);
  
}

TEST_F(DriveGeometryTest, getLineBeforeCircle) {
  Pose2D pose1(0, 0, Angle());
  Pose2D pose2(3, 1, Angle::deg(0));
  Driveable line = DriveGeometry::getLineBeforeCircleGeometry(pose1, pose2);
//   cout << "is valid " << line.isValid() << endl;
  EXPECT_FALSE(line.isValid());
  
  pose1 = Pose2D(0,0, Angle());
  pose2 = Pose2D(0.4, 0.5, Angle::deg(45));
  
  line = DriveGeometry::getLineBeforeCircleGeometry(pose2, pose1);
//   cout << "is valid " << line.isValid() << endl;
  EXPECT_TRUE(line.isValid());
  
  pose1 = Pose2D(-1,-0.5, Angle());
  pose2 = Pose2D(5.4, 1.5, Angle::deg(45));
  
  line = DriveGeometry::getLineBeforeCircleGeometry(pose2, pose1);
//   cout << "is valid " << line.isValid() << " line x: " << line.getEnd()(0) << " y: " << line.getEnd()(1) << " angle: " << line.getAngle().deg() <<endl;
  EXPECT_TRUE(line.isValid());
  ASSERT_NEAR(line.getEnd()(0), 0.571573, 0.001);
  ASSERT_NEAR(line.getEnd()(1), -0.5, 0.001);
  ASSERT_NEAR(line.getAngle().deg() , 0, 0.001);

  
  pose1 = Pose2D(-1, 0.5, Angle());
  pose2 = Pose2D(5.4, -1.5, Angle::deg(-45));
  
  line = DriveGeometry::getLineBeforeCircleGeometry(pose2, pose1);
//   cout << "is valid " << line.isValid() << " line x: " << line.getEnd()(0) << " y: " << line.getEnd()(1) << " angle: " << line.getAngle().deg() <<endl;
  EXPECT_TRUE(line.isValid());
  ASSERT_NEAR(line.getEnd()(0), 0.571573, 0.001);
  ASSERT_NEAR(line.getEnd()(1), 0.5, 0.001);
  ASSERT_NEAR(line.getAngle().deg() , 0, 0.001);
  
  pose1 = Pose2D(-1,-0.5, Angle());
  pose2 = Pose2D(5.4, 0.5, Angle::deg(45));
  
  line = DriveGeometry::getLineBeforeCircleGeometry(pose2, pose1);
//   cout << "is valid " << line.isValid() << " line x: " << line.getEnd()(0) << " y: " << line.getEnd()(1) << " angle: " << line.getAngle().deg() <<endl;
  EXPECT_TRUE(line.isValid());
  ASSERT_NEAR(line.getEnd()(0), 2.98579, 0.001);
  ASSERT_NEAR(line.getEnd()(1), -0.5, 0.001);
  ASSERT_NEAR(line.getAngle().deg() , 0, 0.001);
  
    
  pose1 = Pose2D(-1,0.5, Angle());
  pose2 = Pose2D(5.4, -0.5, Angle::deg(-45));

  line = DriveGeometry::getLineBeforeCircleGeometry(pose2, pose1);
//   cout << "is valid " << line.isValid() << " line x: " << line.getEnd()(0) << " y: " << line.getEnd()(1) << " angle: " << line.getAngle().deg() <<endl;
  EXPECT_TRUE(line.isValid());
  ASSERT_NEAR(line.getEnd()(0), 2.98579, 0.001);
  ASSERT_NEAR(line.getEnd()(1), 0.5, 0.001);
  ASSERT_NEAR(line.getAngle().deg() , 0, 0.001);
  

}

TEST_F(DriveGeometryTest, getCircleBeforeLine) {
  Pose2D pose1(0, 0, Angle());
  Pose2D pose2(3, 1, Angle::deg(0));
  Driveable circle = DriveGeometry::getCircleBeforeLineGeometry(pose1, pose2);
//   cout << "is valid " << circle.isValid() << endl;
  
  pose1 = Pose2D(0,0, Angle());
  pose2 = Pose2D(0.4, 0.5, Angle::deg(45));
  
  circle = DriveGeometry::getCircleBeforeLineGeometry(pose2, pose1);
//   cout << "is valid " << circle.isValid() << endl;
  EXPECT_FALSE(circle.isValid());
  
  pose1 = Pose2D(-2, -1, Angle());
  pose2 = Pose2D(0.4, 0.5, Angle::deg(45));
  
  circle = DriveGeometry::getCircleBeforeLineGeometry(pose2, pose1);
//   cout << "!is valid " << circle.isValid() << " circle " << circle.origin() << " radius " << circle.radius() <<  endl;
  EXPECT_TRUE(circle.isValid());
  ASSERT_NEAR(circle.radius(), 2.17279, 0.001);
  ASSERT_NEAR(circle.origin()(0), -2, 0.001);
  ASSERT_NEAR(circle.origin()(1), 1.17279, 0.001);

    
  pose1 = Pose2D(-2, 1, Angle());
  pose2 = Pose2D(0.4, -0.5, Angle::deg(-45));
  
  circle = DriveGeometry::getCircleBeforeLineGeometry(pose2, pose1);
//   cout << "!is valid " << circle.isValid() << " circle " << circle.origin() << " radius " << circle.radius() <<  endl;
  EXPECT_TRUE(circle.isValid());
  ASSERT_NEAR(circle.radius(), 2.17279, 0.001);
  ASSERT_NEAR(circle.origin()(0), -2, 0.001);
  ASSERT_NEAR(circle.origin()(1), -1.17279, 0.001);

  pose1 = Pose2D(0, 0, Angle());
  pose2 = Pose2D(4, -2, Angle::deg(-91));
  
  circle = DriveGeometry::getCircleBeforeLineGeometry(pose2, pose1);
//   cout << "! test is valid " << circle.isValid() << " circle " << circle.origin() << " radius " << circle.radius() <<  endl;
  EXPECT_FALSE(circle.isValid());

  pose1 = Pose2D(0, 0, Angle());
  pose2 = Pose2D(6, -2, Angle::deg(-135));
  
<<<<<<< HEAD
      
  pose1 = Pose2D(0, 0, Angle());
  pose2 = Pose2D(1,-2, Angle::deg(-100));
  
  circle = DriveGeometry::getCircleBeforeLineGeometry(pose2, pose1);
  cout << "!is valid " << circle.isValid() << " circle " << circle.origin() << " radius " << circle.radius() <<  endl;
//   EXPECT_TRUE(circle.isValid());
//   ASSERT_NEAR(circle.radius(), 2.17279, 0.001);
//   ASSERT_NEAR(circle.origin()(0), -2, 0.001);
//   ASSERT_NEAR(circle.origin()(1), -1.17279, 0.001);
//   

}
*/
}
