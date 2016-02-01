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

class GeometryTest: public ::testing::Test {
protected:
  GeometryTest() {};
  virtual ~GeometryTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};


TEST_F(GeometryTest, slope) {
  Circle2D circle(0, 0, 1);
  Angle result;
  Eigen::Vector2d point = circle.getPointOnCircle(Angle(M_PI*0.3));

  Geometry::slope(circle, point, result);
  ASSERT_NEAR(-36, result.deg(), 0.0000001);

  
  point = circle.getPointOnCircle(Angle(M_PI));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(-90, result.deg(), 0.0000001);


  point = circle.getPointOnCircle(Angle(0));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(90, result.deg(), 0.0000001);

  point = circle.getPointOnCircle(Angle(-0.25 * M_PI));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(45, result.deg(), 0.0000001);

  point = circle.getPointOnCircle(Angle(0.25 * M_PI));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(-45, result.deg(), 0.0000001);
 
  point = circle.getPointOnCircle(Angle(-0.5 * M_PI));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(0, result.deg(), 0.0000001);  
  
  point = circle.getPointOnCircle(Angle(0.5 * M_PI));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(0, result.deg(), 0.0000001);  
  
  point = circle.getPointOnCircle(Angle(0.75 * M_PI));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(45, result.deg(), 0.0000001);  
  
  circle = Circle2D(0.25566599354607311, -1.20060729151351, 1.1987100292106072);
  point = circle.getPointOnCircle(Angle(M_PI / 2 - M_PI / 8 ));
  Geometry::slope(circle, point, result);
  ASSERT_NEAR(-22.5, result.deg(), 0.0000001);  
}
}
