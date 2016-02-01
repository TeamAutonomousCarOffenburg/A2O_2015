#include "gtest/gtest.h"

#include "../a2o/worldmodel/lanedetection/ScanLine.h"

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace A2O;
using namespace std;
using namespace cv;
using namespace Eigen;

namespace {

class ScanLineTest: public ::testing::Test {
protected:
	cv::Rect fullImage;

	ScanLineTest() {fullImage = Rect(1, 1, 638, 238); };
  virtual ~ScanLineTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};

  void drawLine(Mat& img, Line& line) {
	  vector<Point> points;
	  line.getLinePoints(points);
	  for (auto it = points.begin(); it != points.end(); ++it) {
		  img.at<uchar>(*it) = 255;
	  }
  }
};

TEST_F(ScanLineTest, testLaneValid) {

	ScanLine testee(440, 240 - 20, 640 - 1, 240 - 20);
	Mat img(Size(640,480),CV_8UC3,Scalar(255,255,255));
	LineDetection result = testee.findFirstLane(img, fullImage, 1);
	ASSERT_TRUE(result.valid);
}

TEST_F(ScanLineTest, testLaneInvalid) {

	ScanLine testee(440, 240 - 20, 640 - 1, 240 - 20);
	Mat img(Size(640,480),CV_8UC3,Scalar(0,0,0));
	LineDetection result = testee.findFirstLane(img, fullImage, 1);
	ASSERT_FALSE(result.valid);
}

TEST_F(ScanLineTest, testFindFirstLaneProperDirection) {

	ScanLine testee(440, 240 - 20, 640 - 1, 240 - 20);
	Mat img(Size(640,480),CV_8UC3,Scalar(0,0,0));
	Line line(500, 235, 450, 10);
	drawLine(img, line);
	LineDetection result = testee.findFirstLane(img, fullImage, 1);
	ASSERT_TRUE(result.valid);
	ASSERT_EQ(500, result.line.start.x);
	ASSERT_EQ(235, result.line.start.y);
	ASSERT_EQ(450, result.line.end.x);
	ASSERT_EQ(10, result.line.end.y);
}

TEST_F(ScanLineTest, testFindFirstLaneWrongDirection) {

	ScanLine testee(440, 220, 639, 220);
	Mat img(Size(640,480),CV_8UC3,Scalar(0,0,0));
	Line line(420, 235, 550, 150);
	drawLine(img, line);
	LineDetection result = testee.findFirstLane(img, fullImage, 1);
	ASSERT_FALSE(result.valid);
}

TEST_F(ScanLineTest, testFindFirstLaneStopOnM) {

	ScanLine testee(440, 220, 639, 220);
	Mat img(Size(640,480),CV_8UC3,Scalar(0,0,0));
	Line line(500, 235, 480, 200);
	drawLine(img, line);
	Line line2(480, 200, 450, 190);
	drawLine(img, line2);
	LineDetection result = testee.findFirstLane(img, fullImage, 1);
	ASSERT_TRUE(result.valid);
	ASSERT_NEAR(480, result.line.end.x, 7);
	ASSERT_NEAR(200, result.line.end.y, 7);
}

TEST_F(ScanLineTest, testFindFirstLaneBendOnScanline) {

	ScanLine testee(440, 220, 639, 220);
	Mat img(Size(640,480),CV_8UC3,Scalar(0,0,0));
	Line line(440, 220, 410, 200);
	drawLine(img, line);
	Line line2(440, 220, 445, 240);
	drawLine(img, line2);
	LineDetection result = testee.findFirstLane(img, fullImage, 1);
	ASSERT_TRUE(result.valid);
	ASSERT_EQ(440, result.line.start.x);
	ASSERT_EQ(220, result.line.start.y);
}

TEST_F(ScanLineTest, testPixelToCamera) {

	ScanLine testee(0, 0, 10, 10);
	Eigen::Vector3d result = testee.pixelToCamera(Point(320, 480));
	// TODO: verify this value and measure at least one other value
	ASSERT_NEAR(0.25, result(0), 0.1);
	ASSERT_NEAR(0.0, result(1), 0.01);

	Eigen::Vector3d result2 = testee.pixelToCamera(Point(280, 480));
	Eigen::Vector3d result3 = testee.pixelToCamera(Point(360, 480));
	ASSERT_NEAR(result2(1), -result3(1), 0.001);
}


}
