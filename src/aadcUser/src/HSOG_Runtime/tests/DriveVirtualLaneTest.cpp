#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/maneuver/impl/DriveVirtualLane.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class DriveVirtualLaneTest : public ::testing::Test {
protected:
  DriveVirtualLaneTest() {};
  virtual ~DriveVirtualLaneTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};
/*
TEST_F(DriveVirtualLaneTest, getAngle1) {
 Line2D example(0, 0, Angle(M_PI / 2) ,3);
 IManeuverGeometry * geometryPtr = &example;
 
 Angle result; 
 Pose2D pose(0, 0 , Angle(M_PI/2));
 DriveVirtualLane::getAngle(pose, geometryPtr, result);
//  std::cout <<"result:  " << result.deg() << std::endl;
 ASSERT_NEAR(result.deg(), 0, 0.000001);
}

TEST_F(DriveVirtualLaneTest, getAngle2) {
 Line2D example(0, 0, Angle(M_PI / 2) ,3);
 IManeuverGeometry * geometryPtr = &example;
 
 Angle result; 
 Pose2D pose(0.5, 0, Angle(M_PI/2));
 DriveVirtualLane::getAngle(pose, geometryPtr, result);
//  std::cout <<"result:  " << result.deg() << std::endl;
 ASSERT_NEAR(result.deg(), 45, 0.000001);
}

TEST_F(DriveVirtualLaneTest, getAngle3) {
 Line2D example(0, 0, Angle(M_PI / 2) ,3);
 IManeuverGeometry * geometryPtr = &example;
 
 Angle result; 
 Pose2D pose(0.5, 0, Angle(M_PI/4));
 DriveVirtualLane::getAngle(pose, geometryPtr, result);
//  std::cout <<"result:  " << result.deg() << std::endl;
//  ASSERT_NEAR(59.638, result.deg(), 0.000001);
}*/
}