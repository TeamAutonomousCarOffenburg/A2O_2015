#include "gtest/gtest.h"

#include "../a2o/worldmodel/lanedetection/Line.h"
#include "../a2o/utils/geometry/Angle.h"


#include <opencv2/core/core.hpp>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace cv;
using namespace Eigen;

namespace {

class LineTest: public ::testing::Test {
protected:
	LineTest() {};
  virtual ~LineTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};

TEST_F(LineTest, testGetSlope) {

	Line testee1(0,0,10,10);
	ASSERT_NEAR(45, testee1.getSlope().deg(), 0.001);

	Line testee2(0,0,0,10);
	ASSERT_NEAR(90, testee2.getSlope().deg(), 0.001);

	Line testee3(10,0,0,1);
	ASSERT_NEAR(175, testee3.getSlope().deg(), 2);
}

TEST_F(LineTest, testGetLength) {

	Line testee1(0,0,3,4);
	ASSERT_NEAR(5, testee1.getLength(), 0.001);

	Line testee2(0,0,0,10);
	ASSERT_NEAR(10, testee2.getLength(), 0.001);
}

TEST_F(LineTest, testCheckSameDirection) {

	Line testee1(0,0,10,10);
	ASSERT_EQ(0, testee1.checkSameDirection(0,0,10,10, Angle::deg(1)));

	Line testee2(0,0,10,10);
	ASSERT_EQ(1, testee2.checkSameDirection(10,10,0,0, Angle::deg(1)));

	Line testee3(0,0,10,10);
	ASSERT_EQ(-1, testee2.checkSameDirection(0,0,10,11, Angle::deg(1)));

	Line testee4(0,0,10,10);
	ASSERT_EQ(0, testee4.checkSameDirection(0,0,10,9, Angle::deg(10)));

}

TEST_F(LineTest, testGetLinePoints) {

	Line testee1(0,0,5,5);
	vector<Point> points;
	testee1.getLinePoints(points);
	ASSERT_NEAR(6, points.size(), 0.1);
	ASSERT_NEAR(5, points.back().x, 0.1);
	ASSERT_NEAR(5, points.back().y, 0.1);

	Line testee2(5,0,0,0);
	vector<Point> points2;
	testee2.getLinePoints(points2);
	ASSERT_NEAR(6, points2.size(), 0.1);
	ASSERT_NEAR(0, points2.back().x, 0.1);
	ASSERT_NEAR(0, points2.back().y, 0.1);

}

}
