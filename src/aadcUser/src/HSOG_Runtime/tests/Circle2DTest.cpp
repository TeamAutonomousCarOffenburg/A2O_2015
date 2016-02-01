#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/utils/geometry/Pose2D.h"
#include "../a2o/utils/geometry/Circle2D.h"
#include "../a2o/utils/geometry/Geometry.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class Circle2DTest: public ::testing::Test {
protected:
  Circle2DTest() {};
  virtual ~Circle2DTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};


TEST_F(Circle2DTest, TangentConstructor) {
  Circle2D circleTemplate(2, 10, 2);
  Eigen::Vector2d point = circleTemplate.getPointOnCircle(Angle(M_PI * 0.5));
  Eigen::Vector2d tangentPoint = circleTemplate.getPointOnCircle(Angle(M_PI));

  Angle result(0);
  Geometry::slope(circleTemplate, tangentPoint, result);
  const Pose2D tangent(tangentPoint, result);
  Circle2D testee(tangent, point);
//   std::cout << "Circle x: " << testee.origin()(0) << " y: " << testee.origin()(1) << " radius: " << testee.radius() << std::endl;
  ASSERT_NEAR(testee.origin()(0), circleTemplate.origin()(0), 0.000001);
  ASSERT_NEAR(testee.origin()(1), circleTemplate.origin()(1), 0.000001);
  ASSERT_NEAR(testee.radius(), circleTemplate.radius(), 0.000001);
}

TEST_F(Circle2DTest, TangentConstructor2) {
  Circle2D circleTemplate(1, 0, 1);
  Eigen::Vector2d point = circleTemplate.getPointOnCircle(Angle(M_PI / 8));
  Eigen::Vector2d tangentPoint = circleTemplate.getPointOnCircle(Angle(0));

  Angle result(0);
  Geometry::slope(circleTemplate, tangentPoint, result);
//   cout << "result: " << result.deg() << endl;
  const Pose2D tangent(tangentPoint, result);
  Circle2D testee(tangent, point);
  ASSERT_NEAR(testee.origin()(0), circleTemplate.origin()(0), 0.000001);
  ASSERT_NEAR(testee.origin()(1), circleTemplate.origin()(1), 0.000001);
  ASSERT_NEAR(testee.radius(), circleTemplate.radius(), 0.000001);
}

TEST_F(Circle2DTest, TangentConstructor3) {
  Circle2D circleTemplate(4.3, 1.9, 1);
  Eigen::Vector2d point = circleTemplate.getPointOnCircle(Angle(M_PI / 8));
  Eigen::Vector2d tangentPoint = circleTemplate.getPointOnCircle(Angle(M_PI / 2));

  Angle result(0);
  Geometry::slope(circleTemplate, tangentPoint, result);
  const Pose2D tangent(tangentPoint, result);
  Circle2D testee(tangent, point);
  ASSERT_NEAR(testee.origin()(0), circleTemplate.origin()(0), 0.000001);
  ASSERT_NEAR(testee.origin()(1), circleTemplate.origin()(1), 0.000001);
  ASSERT_NEAR(testee.radius(), circleTemplate.radius(), 0.000001);
}

TEST_F(Circle2DTest, TangentConstructor4) {
  Circle2D circleTemplate(2, 2, 2);
  Eigen::Vector2d point = circleTemplate.getPointOnCircle(Angle(0));
  Eigen::Vector2d tangentPoint = circleTemplate.getPointOnCircle(Angle(M_PI));

  Angle result(0);
  Geometry::slope(circleTemplate, tangentPoint, result);
  const Pose2D tangent(tangentPoint, result);
  Circle2D testee(tangent, point);
  ASSERT_NEAR(testee.origin()(0), circleTemplate.origin()(0), 0.000001);
  ASSERT_NEAR(testee.origin()(1), circleTemplate.origin()(1), 0.000001);
  ASSERT_NEAR(testee.radius(), circleTemplate.radius(), 0.000001);
}

TEST_F(Circle2DTest, TangentConstructor5) {
  Circle2D circleTemplate(-2, -2, 2);
  Eigen::Vector2d point = circleTemplate.getPointOnCircle(Angle(M_PI * -0.5));
  Eigen::Vector2d tangentPoint = circleTemplate.getPointOnCircle(Angle(M_PI * 0.5));

  Angle result(0);
  Geometry::slope(circleTemplate, tangentPoint, result);
//   cout << "result: " << result.deg() << endl;
  const Pose2D tangent(tangentPoint, result);
  Circle2D testee(tangent, point);
  ASSERT_NEAR(testee.origin()(0), circleTemplate.origin()(0), 0.000001);
  ASSERT_NEAR(testee.origin()(1), circleTemplate.origin()(1), 0.000001);
  ASSERT_NEAR(testee.radius(), circleTemplate.radius(), 0.000001);
}

TEST_F(Circle2DTest, getPointOnCircle) {
  Circle2D testee(0.5, -0.2, 2.1);
  Eigen::Vector2d point = testee.getPointOnCircle(Angle(M_PI));
//   std::cout << " getPointOnCircletest x: " << point(0) << " y: " << point(1) << std::endl;
  ASSERT_NEAR(point(0), -1.6, 0.000001);
  ASSERT_NEAR(point(1), -0.2, 0.000001);
}


TEST_F(Circle2DTest, isOnCircle) {
  Circle2D testee(1, 0, 2);
  ASSERT_TRUE(testee.isOnCircle(testee.getPointOnCircle(M_PI))); 
  ASSERT_TRUE(testee.isOnCircle(testee.getPointOnCircle(M_PI * 0.5))); 
  ASSERT_TRUE(testee.isOnCircle(testee.getPointOnCircle(Angle(M_PI * 0.3)))); 
  ASSERT_TRUE(testee.isOnCircle(testee.getPointOnCircle(Angle(M_PI * 0.2)))); 
  ASSERT_TRUE(testee.isOnCircle(testee.getPointOnCircle(Angle(M_PI * 0.1)))); 
  ASSERT_TRUE(testee.isOnCircle(testee.getPointOnCircle(Angle(M_PI * 0.4))));
  ASSERT_FALSE(testee.isOnCircle(Eigen::Vector2d(-1.9, 0)));

  
}

// TEST_F(Circle2DTest, slope) {
//   Circle2D testee(1, 0, 2);
//   ASSERT_NEAR(testee.slope(Angle(0)).rad(), -M_PI / 2, 0.000001);
//   ASSERT_NEAR(testee.slope(Angle(M_PI)).rad(), -M_PI / 2, 0.000001);
//   ASSERT_NEAR(testee.slope(Angle(M_PI / 2)).rad(), 0, 0.000001);
//   ASSERT_NEAR(testee.slope(Angle(-M_PI / 2)).rad(), 0, 0.000001);
//   ASSERT_NEAR(testee.slope(Angle(0.75 * M_PI)).rad(), M_PI / 4, 0.000001);
//   ASSERT_NEAR(testee.slope(Angle(-0.75 * M_PI)).rad(), -M_PI / 4, 0.000001);
//   
//   testee= Circle2D(-0.285974, 1.21653 , 1.22113);
//   ASSERT_NEAR(testee.slope(Angle::deg(-90)).deg(), 0, 0.000001);
//   ASSERT_NEAR(testee.slope(Angle::deg(-83.3383)).deg(), 6.6617 , 0.000001);
// 
// }

TEST_F(Circle2DTest, getClosestPose) {
  Circle2D testee(1, 0, 2);
  Pose2D pose = testee.getClosestPose(Eigen::Vector2d(1,4));
  ASSERT_NEAR(pose.x(), 1, 0.000001);
  ASSERT_NEAR(pose.y(), 2, 0.000001);
  ASSERT_NEAR(pose.getAngle().rad(), 0, 0.000001);

  pose = testee.getClosestPose(Eigen::Vector2d(5,0));
  ASSERT_NEAR(pose.x(), 3, 0.000001);
  ASSERT_NEAR(pose.y(), 0, 0.000001);
  ASSERT_NEAR(pose.getAngle().rad(), -M_PI/2, 0.000001);

  pose = testee.getClosestPose(Eigen::Vector2d(4,3));
  ASSERT_NEAR(pose.x(), 2.414214, 0.000001);
  ASSERT_NEAR(pose.y(), 1.414214, 0.000001);
  ASSERT_NEAR(pose.getAngle().rad(), -M_PI/4, 0.000001);

  pose = testee.getClosestPose(Eigen::Vector2d(0,0));
  ASSERT_NEAR(pose.x(), -1, 0.000001);
  ASSERT_NEAR(pose.y(), 0, 0.000001);
  ASSERT_NEAR(pose.getAngle().rad(), -M_PI/2, 0.000001);

  pose = testee.getClosestPose(Eigen::Vector2d(0,-1));
  ASSERT_NEAR(pose.x(), -0.414214, 0.000001);
  ASSERT_NEAR(pose.y(), -1.414214, 0.000001);
  ASSERT_NEAR(pose.getAngle().rad(), -M_PI/4, 0.000001);

  pose = testee.getClosestPose(Eigen::Vector2d(0,1));
  ASSERT_NEAR(pose.x(), -0.414214, 0.000001);
  ASSERT_NEAR(pose.y(), 1.414214, 0.000001);
  ASSERT_NEAR(pose.getAngle().rad(), M_PI/4, 0.000001);
}

TEST_F(Circle2DTest, getAnglefromSegmentLength) {
  Circle2D testee(1, 1, 2);
  Angle angle = testee.getAngle(M_PI);
  ASSERT_NEAR(angle.deg(), 90, 0.000001);
  
  angle = testee.getAngle(-M_PI);
  ASSERT_NEAR(angle.deg(), -90, 0.000001);

  angle = testee.getAngle(6);
  ASSERT_NEAR(angle.deg(), 171.887, 0.001);
  
  angle = testee.getAngle(-5);
  ASSERT_NEAR(angle.deg(), -143.239, 0.001);
  
  angle = testee.getAngle(-5*M_PI);
  ASSERT_NEAR(angle.deg(), -90, 0.001);
  
  angle = testee.getAngle(-5*M_PI);
  ASSERT_NEAR(angle.deg(), -90, 0.001);
}

TEST_F(Circle2DTest, getAnglefromPoint) {
  Circle2D testee(1, 1, 2);
  
  Angle angle = testee.getAngle(Vector2d(2,2));
  ASSERT_NEAR(angle.deg(), 45, 0.000001);
  
  angle = testee.getAngle(Vector2d(0, 2));
  ASSERT_NEAR(angle.deg(), 135, 0.000001);

  angle = testee.getAngle(Vector2d(0, 0));
//     std::cout << "angle: " << angle.deg()<< std::endl;

  ASSERT_NEAR(angle.deg(), -135, 0.001);
  
  angle = testee.getAngle(Vector2d(2,0));
  ASSERT_NEAR(angle.deg(), -45, 0.001);
  
  angle = testee.getAngle(Vector2d(2,1));
  ASSERT_NEAR(angle.deg(), 0, 0.001);
  
  angle = testee.getAngle(Vector2d(0,1));
  ASSERT_NEAR(angle.deg(), -180, 0.001);
  
  angle = testee.getAngle(Vector2d(1,2));
  ASSERT_NEAR(angle.deg(), 90, 0.001);
  
  // TODO: find bug : tryy with 0.9 instead of 1 
  angle = testee.getAngle(Vector2d(1,0));
//   std::cout << "an2gle: " << angle.deg()<< std::endl;
  ASSERT_NEAR(angle.deg(), -90, 0.001);
  
  angle = testee.getAngle(Vector2d(5,4));
  ASSERT_NEAR(angle.deg(), 36.8699, 0.001);
//    std::cout << "angle: " << angle.deg()<< std::endl;
  ASSERT_NEAR(angle.deg(), 36.8699, 0.001);
  
  testee = Circle2D(0.25566599354607311, -1.20060729151351, 1.1987100292106072);
  angle = testee.getAngle(Vector2d(0.17994884617987233,-0.0042910090343237367));
  ASSERT_NEAR(angle.deg(), 93.6215, 0.001);



}

TEST_F(Circle2DTest, getDirectionOfRotation) {
  Circle2D testee(1, 1, 2);
  double direction;
  Angle angle = Angle::deg(110);

  direction = testee.getDirectionOfRotation(Vector2d(2,2), angle);
  ASSERT_NEAR(direction, 1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(2,2), Angle::deg(-45));
  ASSERT_NEAR(direction, -1, 0.000001);
  
  direction = testee.getDirectionOfRotation(Vector2d(1,3), Angle::deg(-180));
  ASSERT_NEAR(direction, 1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(1,3), Angle::deg(180));
  ASSERT_NEAR(direction, 1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(1,3), Angle::deg(0));
  ASSERT_NEAR(direction, -1, 0.000001);

  direction = testee.getDirectionOfRotation(Vector2d(1,-1), Angle::deg(-180));
  ASSERT_NEAR(direction, -1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(1,-1), Angle::deg(180));
  ASSERT_NEAR(direction, -1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(1,-1), Angle::deg(0));
  ASSERT_NEAR(direction, 1, 0.000001);

  direction = testee.getDirectionOfRotation(Vector2d(3,1), Angle::deg(90));
  ASSERT_NEAR(direction, 1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(3,1), Angle::deg(-90));
  ASSERT_NEAR(direction, -1, 0.000001);
  
  direction = testee.getDirectionOfRotation(Vector2d(-1,1), Angle::deg(-90));
  ASSERT_NEAR(direction, 1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(-1,1), Angle::deg(90));
  ASSERT_NEAR(direction, -1, 0.000001);

  direction = testee.getDirectionOfRotation(Vector2d(-1,-1), Angle::deg(-45));
  ASSERT_NEAR(direction, 1, 0.000001);
  direction = testee.getDirectionOfRotation(Vector2d(-1,-1), Angle::deg(135));
  ASSERT_NEAR(direction, -1, 0.000001);

  
  testee = Circle2D(0.25566599354607311, -1.20060729151351, 1.1987100292106072);
  direction = testee.getDirectionOfRotation(Vector2d(0.17994884617987233,-0.0042910090343237367), Angle(3.1177514560075501));
  ASSERT_NEAR(direction, 1, 0.000001);
  
  testee = Circle2D(0.25566599354607311, -1.20060729151351, 1.1987100292106072);
  direction = testee.getDirectionOfRotation(Vector2d(0.17994884617987233,-0.0042910090343237367), Angle(0.1177514560075501));
  ASSERT_NEAR(direction, -1, 0.000001);

  testee = Circle2D(0.25566599354607311, -1.20060729151351, 1.1987100292106072);
  direction = testee.getDirectionOfRotation(Vector2d(0.17994884617987233,-0.0042910090343237367), Angle(-0.1177514560075501));
  ASSERT_NEAR(direction, -1, 0.000001);
  
}

TEST_F(Circle2DTest, getTrail) {
  Circle2D testee(1, 1, 2);
  Vector2d point1 = testee.getPointOnCircle(Angle());
  Vector2d point2 = testee.getPointOnCircle(Angle::deg(45));
//   std::cout << "point 1 x: " << point1(0) << " y: " << point1(1) << endl;
//   std::cout << "point 2 x: " << point2(0) << " y: " << point2(1) << endl;

  
  
  std::vector<Vector2d> points = testee.getTrail(point1, point2);
//   for(Vector2d point : points)
//   {
//     std::cout << "point x: " << point(0) << " y: " << point(1) << endl;
//   }
  ASSERT_NEAR(points[0](0), 3, 0.001);
  ASSERT_NEAR(points[4](0), 2.84776, 0.001);
  ASSERT_NEAR(points[9](0), 2.41421, 0.001);
  ASSERT_NEAR(points.size(), 10, 0.001);


  
  
}
}