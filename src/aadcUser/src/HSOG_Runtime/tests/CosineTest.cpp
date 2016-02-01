#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"
#include "../a2o/utils/geometry/Pose2D.h"
#include "../a2o/utils/geometry/Cosine.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class CosineTest: public ::testing::Test {
protected:
  CosineTest() {};
  virtual ~CosineTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};


TEST_F(CosineTest, getClosedPose) {
  Cosine cosine;
  
  cosine = Cosine();
  Pose2D pose = cosine.getClosestPose(Vector2d(0,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl; 
  ASSERT_NEAR(pose.getAngle().deg(), 0, 0.000001);
  ASSERT_NEAR(pose.x(), 0, 0.000001);
  ASSERT_NEAR(pose.y(), 1, 0.000001);
  
  pose = cosine.getClosestPose(Vector2d(M_PI,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 0, 0.000001);
  ASSERT_NEAR(pose.x(), M_PI, 0.000001);
  ASSERT_NEAR(pose.y(), -1, 0.000001);
  
  pose = cosine.getClosestPose(Vector2d(2* M_PI,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 0, 0.000001);
  ASSERT_NEAR(pose.x(), 2* M_PI, 0.000001);
  ASSERT_NEAR(pose.y(), 1, 0.000001);
  
  
  pose = cosine.getClosestPose(Vector2d(M_PI / 2 ,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl; 
  ASSERT_NEAR(pose.getAngle().deg(), -45, 0.000001);
  ASSERT_NEAR(pose.x(), M_PI / 2, 0.000001);
  ASSERT_NEAR(pose.y(), 0, 0.000001);
  
  pose = cosine.getClosestPose(Vector2d(3 * M_PI / 2 ,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl; 
  ASSERT_NEAR(pose.getAngle().deg(), 45, 0.000001);
  ASSERT_NEAR(pose.x(), 3 * M_PI / 2, 0.000001);
  ASSERT_NEAR(pose.y(), 0, 0.000001);
  
  pose = cosine.getClosestPose(Vector2d(2 ,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl; 
  ASSERT_NEAR(pose.getAngle().deg(), -42.280166819595, 0.001);
  ASSERT_NEAR(pose.x(), 2, 0.000001);
  ASSERT_NEAR(pose.y(), -0.416147, 0.000001);
}


TEST_F(CosineTest, getClosestPoseMovedAndRotatedCosine) {
  Cosine cosine;
  Pose2D origin(1, 1, Angle(0));
  cosine = Cosine(origin, 1, 1);
  Pose2D pose = cosine.getClosestPose(Vector2d(0,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl; 
  ASSERT_NEAR(pose.getAngle().deg(), 40.07963, 0.001);
  ASSERT_NEAR(pose.x(), 0, 0.0001);
  ASSERT_NEAR(pose.y(), 1.5403, 0.0001);
  
  origin = Pose2D(1, 1, Angle(M_PI / 2));
  cosine = Cosine(origin, 1, 1);
  pose = cosine.getClosestPose(Vector2d(0,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 130.08, 0.001);
  ASSERT_NEAR(pose.x(), 0.459698, 0.000001);
  ASSERT_NEAR(pose.y(), 0, 0.000001);
  
  origin = Pose2D(0, 0, Angle());
  cosine = Cosine(origin, 1, 2);
  pose = cosine.getClosestPose(Vector2d(0,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 0, 0.000001);
  ASSERT_NEAR(pose.x(), 0, 0.000001);
  ASSERT_NEAR(pose.y(), 2, 0.000001);
  
  origin = Pose2D(0, 0, Angle());
  cosine = Cosine(origin, 1, -0.5);
  pose = cosine.getClosestPose(Vector2d(0,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 0, 0.000001);
  ASSERT_NEAR(pose.x(), 0, 0.000001);
  ASSERT_NEAR(pose.y(), -0.5, 0.000001);
  
  origin = Pose2D(0, 0, Angle());
  cosine = Cosine(origin, 2, 1);
  pose = cosine.getClosestPose(Vector2d(M_PI,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 0, 0.000001);
  ASSERT_NEAR(pose.x(), M_PI, 0.000001);
  ASSERT_NEAR(pose.y(), 1, 0.000001);
    
  origin = Pose2D(0, 0, Angle());
  cosine = Cosine(origin, 2, 1);
  pose = cosine.getClosestPose(Vector2d(3.0* M_PI / 4.0,0));
//   cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 63.4349, 0.001);
  ASSERT_NEAR(pose.x(), 3.0/4.0 * M_PI, 0.0001);
  ASSERT_NEAR(pose.y(), 0, 0.0001);
  
  origin = Pose2D(0, 0, Angle());
  cosine = Cosine(origin, 2, 1);
  pose = cosine.getClosestPose(Vector2d(3.0* M_PI / 4.0,0));
  cout << "closest pose angle (deg): " << pose.getAngle().deg() << " x: " << pose.x() << " y: " << pose.y() << endl;
  ASSERT_NEAR(pose.getAngle().deg(), 63.4349, 0.001);
  ASSERT_NEAR(pose.x(), 3.0/4.0 * M_PI, 0.0001);
  ASSERT_NEAR(pose.y(), 0, 0.0001);
}

TEST_F(CosineTest, getTrail) {
  Cosine cosine;
  Pose2D origin(1, 1, Angle(0));
  cosine = Cosine(origin, 1, 1);
  std::vector<Vector2d> points= cosine.getTrail(Vector2d(M_PI + 1, 0), Vector2d(1,2) );
//   for(Vector2d point : points)
//   {
//     std::cout << "point x: " << point(0) << " y: " << point(1) << endl;
//   }
  
  ASSERT_NEAR(points[0](0),3.82743, 0.001);
  ASSERT_NEAR(points[4](1), 1, 0.0001);
  ASSERT_NEAR(points[9](0), 1, 0.0001);
}

}