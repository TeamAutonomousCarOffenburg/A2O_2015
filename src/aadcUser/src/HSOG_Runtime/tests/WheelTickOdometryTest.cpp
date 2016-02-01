#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/utils/geometry/Pose2D.h"
#include "../a2o/worldmodel/odometry/WheelTickOdometry.h"
#include "../a2o/utils/geometry/drive/DriveCalculator.h"
#include "../a2o/utils/geometry/drive/DriveGeometry.h"

#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

#define WHEEL_DIAMETER 0.1

class WheelTickOdometryTest: public ::testing::Test {
protected:
  WheelTickOdometryTest() {};
  virtual ~WheelTickOdometryTest() {};
  
  virtual void SetUp() {
  };
  virtual void TearDown() {
  };
};



TEST_F(WheelTickOdometryTest, update) {
  DriveCalculator::Ptr driveCalculator(boost::make_shared<DriveCalculator>());

  WheelTickOdometry testee(driveCalculator, WHEEL_DIAMETER);
  Pose2D resultPose;
  resultPose = testee.update(0, 0, 0);
//   std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
  ASSERT_NEAR(resultPose.x(), 0, 0.000001);
  ASSERT_NEAR(resultPose.y(), 0, 0.000001);
  ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.000001);

  resultPose = testee.update(4,4, 10);
//   std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
  ASSERT_NEAR(resultPose.x(), 0, 0.000001);
  ASSERT_NEAR(resultPose.y(), 0.157079, 0.000001);
  ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.000001);

  resultPose = testee.update(6,6, 20);
//   std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
  ASSERT_NEAR(resultPose.x(), 0, 0.000001);
  ASSERT_NEAR(resultPose.y(), 0.235619, 0.000001);
  ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.000001);


  resultPose = testee.update(8,8, 30);
//   std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
  ASSERT_NEAR(resultPose.x(), 0, 0.000001);
  ASSERT_NEAR(resultPose.y(), 0.314159, 0.000001);
  ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.000001);

  resultPose = testee.update(11,10, 40);
//   std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
  ASSERT_NEAR(resultPose.x(), 0.00739998, 0.000001);
  ASSERT_NEAR(resultPose.y(), 0.411961, 0.000001);
  ASSERT_NEAR(resultPose.getAngle().deg(), -8.653847, 0.000001);

}

TEST_F(WheelTickOdometryTest, testDriveStraight) {
  DriveCalculator::ConstPtr driveCalculator(boost::make_shared<DriveCalculator>(1, 2));
  WheelTickOdometry testee(driveCalculator, WHEEL_DIAMETER);
  
  testee.update(0, 0, 0);
  testee.update(1, 1, 1000);
  
  Pose2D result = testee.getCurrentPose();
  
  ASSERT_NEAR(result.x(), 0, 0.00000001);
  ASSERT_NEAR(result.y(), 0.1 * M_PI / 8, 0.00000001);
  ASSERT_NEAR(result.getAngle().deg(), 0, 0.00000001);
}

TEST_F(WheelTickOdometryTest, testDriveSynchron) {
  DriveCalculator::ConstPtr driveCalculator(boost::make_shared<DriveCalculator>(1, 2));
  WheelTickOdometry testee(driveCalculator, WHEEL_DIAMETER);
  
  testee.update(0, 0, 0);
  testee.update(0, 1, 1000);
  testee.update(1, 2, 2000);
  
  Pose2D result = testee.getCurrentPose();
  Pose2D shouldBe = DriveGeometry::calculateCurvePose(0.1 * M_PI / 8, 0.1 * M_PI / 4, 1);
  
  ASSERT_NEAR(result.x(), shouldBe.x(), 0.00000001);
  ASSERT_NEAR(result.y(), shouldBe.y(), 0.00000001);
  ASSERT_NEAR(result.getAngle().deg(), shouldBe.getAngle().deg(), 0.00000001);
}

TEST_F(WheelTickOdometryTest, testDriveSomehow) {
  DriveCalculator::ConstPtr driveCalculator(boost::make_shared<DriveCalculator>(1, 2));
  WheelTickOdometry testee(driveCalculator, WHEEL_DIAMETER);
  
  testee.update(0, 0, 0);
  testee.update(0, 1, 1000);
  testee.update(0, 2, 2000);
  testee.update(1, 3, 3000);
  
  Pose2D result = testee.getCurrentPose();
  Pose2D shouldBe = DriveGeometry::calculateCurvePose(0.1 * M_PI / 8, 3 * 0.1 * M_PI / 8, 1);
  
  ASSERT_NEAR(result.x(), shouldBe.x(), 0.00000001);
  ASSERT_NEAR(result.y(), shouldBe.y(), 0.00000001);
  ASSERT_NEAR(result.getAngle().deg(), shouldBe.getAngle().deg(), 0.00000001);
}

TEST_F(WheelTickOdometryTest, testDriveSomehow2) {
  DriveCalculator::ConstPtr driveCalculator(boost::make_shared<DriveCalculator>(1, 2));
  WheelTickOdometry testee(driveCalculator, WHEEL_DIAMETER);
  
  testee.update(0, 0, 0);
  testee.update(1, 0, 1500);
  testee.update(0, 1, 2000);
  testee.update(2, 1, 3000);
  testee.update(0, 2, 4000);
  testee.update(3, 2, 4500);
  testee.update(4, 3, 6000);
  
  Pose2D result = testee.getCurrentPose();
  Pose2D shouldBe = DriveGeometry::calculateCurvePose(0.1 * M_PI / 2, 3 * 0.1 * M_PI / 8, 1);
  
  ASSERT_NEAR(result.x(), shouldBe.x(), 0.00000001);
  ASSERT_NEAR(result.y(), shouldBe.y(), 0.00000001);
  ASSERT_NEAR(result.getAngle().deg(), shouldBe.getAngle().deg(), 0.00000001);
}
}
