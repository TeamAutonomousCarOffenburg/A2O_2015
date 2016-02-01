#pragma once

#include "ISegmentDetection.h"
#include "StraightDetection.h"
#include "Curve2Detection.h"
#include "CrossingDetection.h"
#include "Segment.h"

#include <vector>


namespace A2O {

class SegmentDetector {
public:
  SegmentDetector();
  ~SegmentDetector();
  
  const std::vector<StraightDetection>& getStraightDetections() const;
  const std::vector<Curve2Detection>& getCurveDetections() const;
  const std::vector<CrossingDetection>& getCrossingDetections() const;

  void addStraightDetection(Segment::Ptr seg,
			    const Pose2D& pose);
  void addTCrossingLSDetection(Segment::Ptr seg,
			       const Pose2D& pose);
  void addTCrossingLRDetection(Segment::Ptr seg,
			       const Pose2D& pose);
  void addTCrossingSRDetection(Segment::Ptr seg,
			       const Pose2D& pose);
  void addXCrossingDetection(Segment::Ptr seg,
			     const Pose2D& pose);
  
  void addCrossingDetection(Segment::Ptr seg,
			    const Pose2D& pose,
			    const CrossingType& type);
  void addCurveDetection(Segment::Ptr seg,
			 const Circle2D& circle,
			 const bool& leftCurve);
  
  void progressTTL();
  void clearDeadDetections();
  
  void reset();
  
  const bool redefineNextSegment(Segment::Ptr seg) const;
  
private:
  const bool redefineStraight(Segment::Ptr seg,
			      const StraightDetection& straightDetection) const;
  const bool redefineCurve(Segment::Ptr seg,
			   const Curve2Detection& curveDetection) const;
  const bool redefineCrossing(Segment::Ptr seg,
			      const CrossingDetection& crossingDetection) const;
  
  std::vector<StraightDetection> _straightDetections;
  std::vector<Curve2Detection> _curveDetections;
  std::vector<CrossingDetection> _crossingDetections;
};

}
