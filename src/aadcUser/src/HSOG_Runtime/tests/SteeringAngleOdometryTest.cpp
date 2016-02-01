#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/utils/geometry/Pose2D.h"
#include "../a2o/worldmodel/odometry/SteeringAngleOdometry.h"

#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class SteeringAngleOdometryTest: public ::testing::Test {
protected:
  SteeringAngleOdometryTest() {};
  virtual ~SteeringAngleOdometryTest() {};
  
  virtual void SetUp() {
  };
  virtual void TearDown() {
  };
};


// TODO: Refine Test
// TEST_F(SteeringAngleOdometryTest, update) {
//   SteeringAngleOdometry testee;
// //   double wheelTick = 0;
// //   double steeringAngle = 0;
// //   long time = 0;
// //   testee.update(wheelTick, steeringAngle, time);
//   Pose2D resultPose;
//   // const double& WheelTicks, const double& steeringAngle, const long& time
//   resultPose = testee.update(0, 0, 0);
// //   std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), 0, 0.000001);
//   ASSERT_NEAR(resultPose.y(), 0, 0.000001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.000001);
// 
//   
//   resultPose = testee.update(4,0, 10);
// //  std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), 0.136659, 0.000001);
//   ASSERT_NEAR(resultPose.y(), 0, 0.000001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.000001);
// 
// resultPose = testee.update(10, 0, 20);
// //   std::cout << "result2: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), 0.341648, 0.000001);
//   ASSERT_NEAR(resultPose.y(), 0, 0.000001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.001);
// 
// 
//   resultPose = testee.update(20, -20, 30);
// //   std::cout << "result: rechts angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), 0.670768, 0.000001);
//   ASSERT_NEAR(resultPose.y(), -0.122284, 0.000001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), -13.2643, 0.001);
// 
//   resultPose = testee.update(30, 20, 40);
// //   std::cout << "result: links angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), 1.01916, 0.00001);
//   ASSERT_NEAR(resultPose.y(), -0.0787768, 0.00001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.001);
// 
//   
//   resultPose = testee.update(40, 20, 40);
// //   std::cout << "result: links  angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), 1.34828, 0.00001);
//   ASSERT_NEAR(resultPose.y(), 0.0435069, 0.00001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), 13.2643, 0.001);
//   
//   resultPose = testee.update(50, -20, 40);
// //   std::cout << "result: rechts angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), 1.69668, 0.00001);
//   ASSERT_NEAR(resultPose.y(), 0, 0.00001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), 0, 0.001);
// 
// }

// TEST_F(SteeringAngleOdometryTest, update2) {
// 
//   const double diameter = 0.105;
//   const double frontAxleLength = 0.24;
//   const double rearAxleLength = 0.26;
//   const Eigen::Vector3d frontAxle (0.18,0,0);
//   const Eigen::Vector3d rearAxle (-0.18, 0 ,0);
//   
//   SteeringAngleOdometry* testee = new SteeringAngleOdometry(diameter, frontAxle, frontAxleLength, rearAxle, rearAxleLength);
//   Pose2D resultPose;
//   Angle angle = Angle::deg(178.634);
//   Pose2D pose(0, 0, angle);
//   testee->setCurrentPose(pose);
//     resultPose = testee->update(0, 0, 0);
// //   std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
// 
//   resultPose = testee->update(1, 23.3721, 10);
// //   std::cout << "3, 25 result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), -0.0350332, 0.000001);
//   ASSERT_NEAR(resultPose.y(), -0.0238322, 0.000001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), -174.664, 0.01);
// }

TEST_F(SteeringAngleOdometryTest, update3) {

  const double diameter = 0.105;
  const double frontAxleLength = 0.24;
  const Eigen::Vector3d frontAxle (0.18,0,0);
  
  SteeringAngleOdometry* testee = new SteeringAngleOdometry(diameter, frontAxle, frontAxleLength, frontAxleLength);
  Pose2D resultPose;
  Angle angle = Angle::deg(0);
  Pose2D pose(0.0, 0.0, angle);
  testee->setCurrentPose(pose);
    resultPose = testee->update(0, 0);
  std::cout << "result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 

  resultPose = testee->update(1, 23.0233);
  std::cout << "3, 25 result: angle: " << resultPose.getAngle().deg() << " x: " << resultPose.x() << " y: " << resultPose.y() << std::endl; 
//   ASSERT_NEAR(resultPose.x(), -0.0350332, 0.000001);
//   ASSERT_NEAR(resultPose.y(), -0.0238322, 0.000001);
//   ASSERT_NEAR(resultPose.getAngle().deg(), -174.664, 0.01);
}
}
