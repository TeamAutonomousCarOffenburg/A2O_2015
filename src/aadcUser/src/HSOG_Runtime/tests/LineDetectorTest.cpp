#include "gtest/gtest.h"
 
#include "../a2o/worldmodel/lanedetection/openCvLane.h"
#include "../a2o/worldmodel/lanedetection/Line.h"
#include "../a2o/worldmodel/lanedetection/LineDetection.h"
#include "../a2o/worldmodel/lanedetection/CurveDetection.h"
#include "../a2o/worldmodel/lanedetection/RoadStatus.h"
#include "../a2o/worldmodel/lanedetection/LineDetector.h"
 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
 
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <string>
#include <dirent.h>
#include <stdlib.h>
 
 
using namespace A2O;
using namespace std;
using namespace cv;
using namespace Eigen;
 
bool ends_with(std::string const & value, std::string const & ending)
{
      if (ending.size() > value.size()) return false;
      return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}
 
int filter(const struct dirent *entry)
{
        return ends_with (entry->d_name, ".png") || ends_with (entry->d_name, ".jpeg");
}
 
namespace {
 
class LineDetectorTest: public ::testing::Test {
protected:
        LineDetectorTest() {};
  virtual ~LineDetectorTest() {};
 
  virtual void SetUp() {};
  virtual void TearDown() {};
 
  const Mat getMatFromImage(string path)
  {
    const Mat image = imread(path.data(), CV_LOAD_IMAGE_COLOR);   // Read the file
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl;
    }
    return image;
  }
  void drawCurve(Mat& image, CurveDetection curve, Scalar& color)
  {
          line(image, curve.start, curve.middle, color);
          line(image, curve.middle, curve.end, color);
  }
 
  void drawLine(Mat& image, Line line2, Scalar& color)
  {
          line(image, line2.start, line2.end, color);
  }
 
  void drawTestee(const Mat& image, const Mat& edges, LineDetector testee) {
                Rect rect(0, 240, 639, 240);
                Mat temp_frame = image(rect);
                Scalar red = Scalar(0, 0, 255);
                Scalar green = Scalar(0, 255, 0);
                Scalar blue = Scalar(255, 0, 0);
                Scalar yellow = Scalar(0, 255, 255);
                Scalar mag = Scalar(255, 0, 255);
                Scalar orange = Scalar(0, 130, 200);
 
                if (testee.rightLine.valid)
                        drawLine(temp_frame, testee.rightLine.line, red);
                if (testee.stopLine.valid)
                        drawLine(temp_frame, testee.stopLine.line, green);
                if (testee.stopLineMiddle.valid)
                        drawLine(temp_frame, testee.stopLineMiddle.line, red);
                if (testee.startRight.valid)
                        drawLine(temp_frame, testee.startRight.line, red);
                if (testee.endRight.valid)
                        drawLine(temp_frame, testee.endRight.line, red);
                if (testee.stopRightMiddle.valid)
                        drawLine(temp_frame, testee.stopRightMiddle.line, blue);
                if (testee.stopRight.valid)
                        drawLine(temp_frame, testee.stopRight.line, blue);
                if (testee.upperRightStart.valid)
                        drawLine(temp_frame, testee.upperRightStart.line, mag);
                if (testee.upperRight.valid)
                        drawLine(temp_frame, testee.upperRight.line, mag);
 
                if (testee.startLeft.valid)
                        drawLine(temp_frame, testee.startLeft.line, mag);
                if (testee.leftLine.valid)
                        drawLine(temp_frame, testee.leftLine.line, mag);
                if (testee.endLeft.valid)
                        drawLine(temp_frame, testee.endLeft.line, mag);
                if (testee.stopLeftMiddle.valid)
                        drawLine(temp_frame, testee.stopLeftMiddle.line, blue);
                if (testee.stopLeft.valid)
                        drawLine(temp_frame, testee.stopLeft.line, blue);
                if (testee.upperLeft.valid)
                        drawLine(temp_frame, testee.upperLeft.line, green);
                if (testee.upperLeftStart.valid)
                        drawLine(temp_frame, testee.upperLeftStart.line, green);
 
                if (testee.rightCurve.valid) {
                        drawCurve(temp_frame, testee.rightCurve, yellow);
                        string radiusStr("radiusRight: " + to_string(testee.rightCurve.radius));
                        putText(temp_frame, radiusStr,
                                        cvPoint(10,15), FONT_HERSHEY_COMPLEX_SMALL, 0.8, blue, 1, CV_AA);
                }
                if (testee.leftCurve.valid) {
                        drawCurve(temp_frame, testee.leftCurve, yellow);
                        string radiusStr("radiusLeft: " + to_string(testee.leftCurve.radius));
                        putText(temp_frame, radiusStr,
                                        cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, blue, 1, CV_AA);
                }
 
                putText(temp_frame, RoadStatusName::getNameForRoadStatus(testee.nextSegment),
                                cvPoint(10, 45), FONT_HERSHEY_COMPLEX_SMALL, 0.8, blue, 1, CV_AA);
 
                // draw static Scanlines
                Line scan(340, 220, 639, 220);
                drawLine(temp_frame, scan, orange);
 
                scan = Line(350, 75, 450, 75);
                drawLine(temp_frame, scan, orange);
 
                scan = Line(8, 180, 8, 50);
                drawLine(temp_frame, scan, orange);
 
                namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
                imshow("Display window", temp_frame); // Show our image inside it.
 
                namedWindow("Edge window", WINDOW_AUTOSIZE); // Create a window for display.
                imshow("Edge window", edges); // Show our image inside it.
 
                waitKey(0);
        }
 
};
 
TEST_F(LineDetectorTest, testLine) {
 
        Mat image;
//      Mat image = getMatFromImage("XCROSSROAD.png");
//      Mat image = getMatFromImage("TRIGHT.png");
//      Mat image = getMatFromImage("TLEFT.png");
//      Mat image = getMatFromImage("SCRIGHT.png");
//      Mat image = getMatFromImage("BCRIGHT.png");
//      Mat image = getMatFromImage("SCLEFT.png");
//      Mat image = getMatFromImage("BCLEFT.png");
//      Mat image = getMatFromImage("TLEFTRIGHT.png");
//      Mat image = getMatFromImage("TLRFAR.jpeg");
//      Mat image = getMatFromImage("STRAIGHT.jpeg");
 
        if (!image.data) {
                return;
        }
 
        LaneDetection opencv;
        Mat edges = opencv.detectEdges(image);
 
        LineDetector testee;
        testee.detectLanes(edges);
        ASSERT_TRUE(testee.rightLine.valid);
 
        drawTestee(image, edges, testee);
}
 
TEST_F(LineDetectorTest, testWholeDirectory) {
 
    DIR *dp;
    struct dirent **dirp;
    string dir = "doesnotexistforskipingthis";
    // string dir = "bilder/laengs_ohne/";
    // string dir = "bilder/laengs_mit/";
    // string dir = "bilder/quer/";
    // string dir = "bilder/TLane/";
    // string dir = "bilder/video1part1/";
    // string dir = "bilder/teststrecke/";
    // string dir = "bilder/teststrecke2/";
    // string dir = "bilder/testLight/";
    // string dir = "bilder/tright/";
    // string dir = "bilder/tleft/";
    // string dir = "bilder/curve/";
    // string dir = "bilder/singlePicture/";
 
    if((dp  = opendir(dir.c_str())) == NULL) {
        return;
    }
 
    int fileCount = scandir(dir.c_str(), &dirp, filter, alphasort);
 
    for(int i = 0; i < fileCount; i++)
    {
        string file = dir + string(dirp[i]->d_name);
        std::cout << file << std::endl;
        Mat image = getMatFromImage(file);
        if (!image.data) {
                closedir(dp);
                return;
        }
 
        LaneDetection opencv;
        Mat edges = opencv.detectEdges(image);
 
        LineDetector testee;
        // testee.initialDetection(edges);
        testee.detectLanes(edges);
 
        drawTestee(image, edges, testee);
     }
     free(dirp);
     closedir(dp);
}
 
TEST_F(LineDetectorTest, testAllLines) {
 
        LaneDetection opencv;
        int images = 10;
        const string names[] = {"XCROSSROAD.png", "TRIGHT.png", "TLEFT.png", "SCRIGHT.png", "BCRIGHT.png",
                        "SCLEFT.png", "BCLEFT.png", "TLEFTRIGHT.png", "TLRFAR.jpeg", "STRAIGHT.jpeg"};
        for (int i = 0; i < images; i++) {
                Mat image = getMatFromImage(names[i]);
                if (!image.data) {
                        return;
                }
 
                Mat edges = opencv.detectEdges(image);
                LineDetector testee;
                testee.detectLanes(edges);
 
                // drawTestee(image, edges, testee);
        }
}
 
TEST_F(LineDetectorTest, testInitialAllignment) {
 
        Mat image = getMatFromImage("PARKSTR.jpeg");
 
        if (!image.data) {
                return;
        }
 
        LaneDetection opencv;
        Mat edges = opencv.detectEdges(image);
 
        LineDetector testee;
        testee.initialDetection(edges);
 
        // drawTestee(image, edges, testee);
}


}
