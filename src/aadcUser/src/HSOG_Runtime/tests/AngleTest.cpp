#include "gtest/gtest.h"

#include "../a2o/utils/geometry/Angle.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace Eigen;

namespace {

class AngleTest: public ::testing::Test {
protected:
  AngleTest() {};
  virtual ~AngleTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};

TEST_F(AngleTest, testGeneralMethods) {
  // Test constructors and getters
  Angle angle1 = Angle::rad(M_PI / 2);
  Angle angle2 = Angle::deg(90);
  Angle angle3;
  Angle angle4(M_PI / 2);
  Angle angle5(angle1);
  Angle angle6(0);
  
  // Test getters
  ASSERT_NEAR(angle1.rad(), angle2.rad(), 0.0000001);
  ASSERT_NEAR(angle1.deg(), angle2.deg(), 0.0000001);
  
  ASSERT_NEAR(angle1.rad(), angle4.rad(), 0.0000001);
  ASSERT_NEAR(angle1.deg(), angle4.deg(), 0.0000001);
  
  ASSERT_NEAR(angle1.rad(), angle5.rad(), 0.0000001);
  ASSERT_NEAR(angle1.deg(), angle5.deg(), 0.0000001);
  
  ASSERT_NEAR(angle3.rad(), angle6.rad(), 0.0000001);
  ASSERT_NEAR(angle3.deg(), angle6.deg(), 0.0000001);
  
  // Test get Deg
  ASSERT_NEAR(angle1.deg(), angle2.rad() * 180 / M_PI, 0.0000001);
  ASSERT_NEAR(angle3.deg(), angle3.rad() * 180 / M_PI, 0.0000001);
  
  // Test assignments
  angle1 = Angle::deg(30);
  angle3 = Angle::rad(M_PI / 6);
  ASSERT_NEAR(angle1.rad(), angle3.rad(), 0.0000001);
  
  angle1 += Angle::deg(30);
  ASSERT_NEAR(angle1.rad(), (M_PI / 3), 0.0000001);
  
  angle3 -= Angle::deg(15);
  ASSERT_NEAR(angle3.rad(), (M_PI / 12), 0.00000001);
  
  angle3 -= angle1;
  ASSERT_NEAR(angle3.rad(), -(M_PI / 4), 0.00000001);
  
  // Test border cases
  angle1 = Angle::deg(179);
  angle1 += Angle::deg(2);
  ASSERT_NEAR(angle1.deg(), -179, 0.00000001);
  
  angle3 = Angle::deg(-179);
  angle3 -= Angle::deg(10);
  ASSERT_NEAR(angle3.deg(), 171, 0.00000001);
}

TEST_F(AngleTest, testBorders) {
  // Test constructors with too big angles
  Angle angle1 = Angle::rad(3 * (M_PI / 2));
  Angle angle2 = Angle::deg(369);
  Angle angle3(5 * (M_PI / 2));
  
  ASSERT_NEAR(angle1.rad(), -(M_PI / 2), 0.00000001);
  ASSERT_NEAR(angle2.deg(), 9, 0.00000001);
  ASSERT_NEAR(angle3.rad(), (M_PI / 2), 0.00000001);
  
  // Test constructors with too small angles
  angle1 = Angle::rad(-3 * (M_PI / 2));
  angle2 = Angle::deg(-369);
  angle3 = Angle::rad(-5 * (M_PI / 2));
  
  // Test getters
  ASSERT_NEAR(angle1.rad(), (M_PI / 2), 0.00000001);
  ASSERT_NEAR(angle2.deg(), -9, 0.00000001);
  ASSERT_NEAR(angle3.rad(), -(M_PI / 2), 0.00000001);
}

TEST_F(AngleTest, testHelperMethods) {
  Angle angle1 = Angle::rad(M_PI);
  Angle angle2 = Angle::deg(45);
  
  ASSERT_NEAR(angle2.negate().rad(), -(M_PI / 4), 0.00000001);
  ASSERT_NEAR(angle2.opposite().rad(), -(3 * M_PI / 4), 0.00000001);
  
  Angle angle3(angle2.opposite());
  ASSERT_TRUE(angle3.isOppositeOf(angle2));
  ASSERT_FALSE(angle3.isOppositeOf(angle1));
  ASSERT_FALSE(angle3.isOppositeOf(angle3));
}

TEST_F(AngleTest, testRotate) {
  Vector2d v1(5, 13);
  Vector2d v2(-11, 7);
  
  // Test toation around +180 deg
  Angle angle1 = Angle::rad(M_PI);
  Vector2d res1 = angle1 * v1;
  ASSERT_NEAR(res1(0), -5, 0.0000001);
  ASSERT_NEAR(res1(1), -13, 0.0000001);
  
  // Test toation around +90 deg
  angle1 = Angle::deg(90);
  res1 = angle1 * v2;
  ASSERT_NEAR(res1(0), -7, 0.0000001);
  ASSERT_NEAR(res1(1), -11, 0.0000001);
  
  // Test toation around -90 deg
  angle1 = Angle::deg(-90);
  res1 = angle1 * v2;
  ASSERT_NEAR(res1(0), 7, 0.0000001);
  ASSERT_NEAR(res1(1), 11, 0.0000001);
}

TEST_F(AngleTest, testEq) {
	  Angle precision = Angle::deg(5);
	  Angle a1 = Angle::deg(1);
	  Angle a2 = Angle::deg(-1);
	  ASSERT_TRUE(a1.eq(a2, precision));

	  a1 = Angle::deg(1);
	  a2 = Angle::deg(7);
	  ASSERT_FALSE(a1.eq(a2, precision));

	  a1 = Angle::deg(179);
	  a2 = Angle::deg(-179);
	  ASSERT_TRUE(a1.eq(a2, precision));
}
}
