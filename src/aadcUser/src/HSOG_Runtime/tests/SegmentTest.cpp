#include "gtest/gtest.h"

#include "../a2o/worldmodel/impl/WorldModel.h"
#include "../a2o/worldmodel/street/Segment.h"
#include "../a2o/worldmodel/street/SegmentLink.h"

using namespace A2O;
using namespace std;

namespace {

class SegmentTest: public ::testing::Test {
protected:
  SegmentTest() {};
  virtual ~SegmentTest() {};
  
  virtual void SetUp() {};
  virtual void TearDown() {};
};

TEST_F(SegmentTest, testgetNextOrCurrentSegmentFromPoseCurrentSegment) {
  
  Pose2D startPose(0,0);
  Segment::Ptr testee = Segment::createInitialSegment(startPose, 1);
 
  SegmentLink::Ptr endLink = testee->getStraightOption();
  testee->appendXCrossing(endLink);
  
  
  Segment::Ptr xcross = endLink->getSegmentAfter();
  
  // middle
  Segment::Ptr t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5,0));
  ASSERT_TRUE(t == xcross);
  
  // left
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5,0.49));
  ASSERT_TRUE(t == xcross); 
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5,0.5));
  ASSERT_TRUE(t == xcross);
   t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5,0.51));
  ASSERT_FALSE(t == xcross);

  // right
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5,-0.49));
  ASSERT_TRUE(t == xcross);
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5,-0.5));
  ASSERT_TRUE(t == xcross);
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5,-0.51));
  ASSERT_FALSE(t == xcross);

  // top
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.99,0));
  ASSERT_TRUE(t == xcross);
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(2,0));
  ASSERT_TRUE(t == xcross);
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(2.01,0));
  ASSERT_FALSE(t == xcross);
  
  // bottom
   t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.01,0));
  ASSERT_TRUE(t == xcross);
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1,0));
  ASSERT_TRUE(t == xcross);
  
  // before alsor returns the currentSegment
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(0.9,0));
  ASSERT_TRUE(t == xcross);
  
  // top right
   t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(2.01,0.51));
  ASSERT_FALSE(t == xcross);
 
}

TEST_F(SegmentTest, testgetNextOrCurrentSegmentFromPoseNextSegments) {
  
  Pose2D startPose(0,0);
  Segment::Ptr testee = Segment::createInitialSegment(startPose, 1);
 
  SegmentLink::Ptr endLink = testee->getStraightOption();
  testee->appendXCrossing(endLink);
  
  Segment::Ptr xcross = endLink->getSegmentAfter();
  
  xcross->appendTCrossingLR(xcross->getLeftOption());
  xcross->appendTCrossingLR(xcross->getRightOption());
  xcross->appendTCrossingLR(xcross->getStraightOption());

  
  Segment::Ptr left = xcross->getLeftOption()->getSegmentAfter();
  Segment::Ptr right = xcross->getRightOption()->getSegmentAfter();
  Segment::Ptr straight = xcross->getStraightOption()->getSegmentAfter();
  
  
  // straight
  Segment::Ptr t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(2.01,0.49));
  ASSERT_TRUE(t == straight);
  
  // left
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5, 1.01));
  ASSERT_TRUE(t == left);
  
  
  // right
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5, -1.01));
  ASSERT_TRUE(t == right);

  // bottom before, returns currentSegment
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(-1, 0));
  ASSERT_TRUE(t == xcross);

}

TEST_F(SegmentTest, testgetNextOrCurrentSegmentFromPoseNextSegmentsRotated) {
  
  Pose2D startPose(0,0,Angle::deg(90));
  Segment::Ptr testee = Segment::createInitialSegment(startPose, 1);
 
  SegmentLink::Ptr endLink = testee->getStraightOption();
  testee->appendXCrossing(endLink);
  
  Segment::Ptr xcross = endLink->getSegmentAfter();
  
  xcross->appendTCrossingLR(xcross->getLeftOption());
  xcross->appendTCrossingLR(xcross->getRightOption());
  xcross->appendTCrossingLR(xcross->getStraightOption());

  
  Segment::Ptr left = xcross->getLeftOption()->getSegmentAfter();
  Segment::Ptr right = xcross->getRightOption()->getSegmentAfter();
  Segment::Ptr straight = xcross->getStraightOption()->getSegmentAfter();
  
  
  // straight
  Segment::Ptr t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(0,2.01));
  ASSERT_TRUE(t == straight);
  
  // left
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(-1.5, 1.5));
  ASSERT_TRUE(t == left);
  
  
  // right
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(1.5, 1.01));
  ASSERT_TRUE(t == right);

  // bottom
  t = Segment::getNextOrCurrentSegmentFromPose(xcross, Pose2D(0, 0));
  ASSERT_TRUE(t == xcross);

}


TEST_F(SegmentTest, testgetSegmentFromSegmentStraight) {
  Pose2D startPose(0,0);
  Segment::Ptr testee = Segment::createInitialSegment(startPose, 1);
 
  SegmentLink::Ptr endLink = testee->getStraightOption();
  testee->appendXCrossing(endLink);
  
  Segment::Ptr xcross = endLink->getSegmentAfter();  
  
  Segment::Ptr t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0,0.5));
  ASSERT_TRUE(t == testee);
  
  t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(1.2,0.5));
  ASSERT_FALSE(t == testee);
  
  
 testee = Segment::createInitialSegment(startPose, 1, Angle::Deg_N90());
 t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0,-0.5));
 ASSERT_TRUE(t == testee);
  
}


TEST_F(SegmentTest, testgetSegmentFromSegmentCurve) {
  Pose2D startPose(0,0);
  Segment::Ptr testee = Segment::createInitialSegment(startPose, 1, Angle::Deg_90());
 
  SegmentLink::Ptr endLink = testee->getStraightOption();
  testee->appendXCrossing(endLink);
  
  Segment::Ptr xcross = endLink->getSegmentAfter();  
  
  Segment::Ptr t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0,0.5));
  ASSERT_TRUE(t == testee);
  
  t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0.3, 0.3));
  ASSERT_TRUE(t == testee);
  
  t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0.5, 0.5));
  ASSERT_TRUE(t == testee);
  
  t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0.7, 0.7));
  ASSERT_TRUE(t == xcross);

  
}


TEST_F(SegmentTest, testgetSegmentFromSegmentNegativeCurve) {
  Pose2D startPose(0,0);
  Segment::Ptr testee = Segment::createInitialSegment(startPose, 1, Angle::Deg_N90());
 
  SegmentLink::Ptr endLink = testee->getStraightOption();
  testee->appendXCrossing(endLink);
  
  Segment::Ptr xcross = endLink->getSegmentAfter();  
  
  Segment::Ptr t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0,0.5));
  ASSERT_TRUE(t == testee);
  
  t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(-0.3, -0.3));
  ASSERT_TRUE(t == testee);
  
   t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0.3, -0.3));
  ASSERT_TRUE(t == testee);
  
  t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0.63, -0.7));
  ASSERT_TRUE(t == xcross);
}


TEST_F(SegmentTest, testgetStraightThenCurve) {
  Pose2D startPose(0,0);
  Segment::Ptr testee = Segment::createInitialSegment(startPose, 1);
 
  SegmentLink::Ptr endLink = testee->getStraightOption();
  testee->appendSegment(endLink, 1, Angle::deg(-60));
  
  Segment::Ptr curve = testee->getStraightOption()->getSegmentAfter();
  curve->appendXCrossing(curve->getStraightOption()); 

  Segment::Ptr xcross = curve->getStraightOption()->getSegmentAfter();
   
  
  Segment::Ptr t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(0,0.5));
  ASSERT_TRUE(t == testee);
  
  t = Segment::getNextOrCurrentSegmentFromPose(testee, Pose2D(1.5, -0.5));
  ASSERT_TRUE(t == curve);
  
  t = Segment::getNextOrCurrentSegmentFromPose(curve, Pose2D(2, -2));
  ASSERT_TRUE(t == xcross);
}
  
  
}
