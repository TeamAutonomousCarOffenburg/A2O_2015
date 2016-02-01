#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Line2D.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class Line2DTest: public ::testing::Test {
protected:
  Line2DTest() {};
  virtual ~Line2DTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {}
};

TEST_F(Line2DTest, getClosestPose) {
  Line2D testee(0,0,Angle(0), 10);
  Pose2D result = testee.getClosestPose(Vector2d(1,1));
  ASSERT_NEAR(result.getPosition()(0), 1, 0.000001);
  ASSERT_NEAR(result.getPosition()(1), 0, 0.000001);
  ASSERT_NEAR(result.getAngle().rad(), 0, 0.000001);

  result = testee.getClosestPose(Vector2d(2,2));
  ASSERT_NEAR(result.getPosition()(0), 2, 0.000001);
  ASSERT_NEAR(result.getPosition()(1), 0, 0.000001);
  ASSERT_NEAR(result.getAngle().rad(), 0, 0.000001);

  Line2D testee2(-1.0, -1.0, Angle(M_PI / 4), 10.0);
  result = testee2.getClosestPose(Vector2d(2 , 0));
  ASSERT_NEAR(result.getPosition()(0), 1, 0.000001);
  ASSERT_NEAR(result.getPosition()(1), 1, 0.000001);
  ASSERT_NEAR(result.getAngle().rad(), M_PI * 0.25, 0.000001);
  
  result = testee2.getClosestPose(Vector2d(0 , 0));
  ASSERT_NEAR(result.getPosition()(0), 0, 0.000001);
  ASSERT_NEAR(result.getPosition()(1), 0, 0.000001);
  ASSERT_NEAR(result.getAngle().rad(), M_PI * 0.25, 0.000001);
};

TEST_F(Line2DTest, getTrail) {
  Line2D testee(1,1, Angle::deg(-45));
  std::vector<Vector2d> points = testee.getTrail(Vector2d(1,1) , Vector2d(3,-1.1));
//     for(Vector2d point : points)
//   {
//     std::cout << "point x: " << point(0) << " y: " << point(1) << endl;
//   }
  ASSERT_NEAR(points.size(), 10, 0.001);
  ASSERT_NEAR(points[0](0),1.205, 0.001);
  ASSERT_NEAR(points[4](1), -0.025, 0.0001);
  ASSERT_NEAR(points[9](0), 3.05, 0.0001);

}

}