#include "SegmentDetector.h"


using namespace A2O;
using namespace Eigen;

#define MIN_DETECTIONS_COUNT 0

SegmentDetector::SegmentDetector()
{
}

SegmentDetector::~SegmentDetector()
{
}

const std::vector<StraightDetection>& SegmentDetector::getStraightDetections() const
{
  return _straightDetections;
}

const std::vector<Curve2Detection>& SegmentDetector::getCurveDetections() const
{
  return _curveDetections;
}

const std::vector<CrossingDetection>& SegmentDetector::getCrossingDetections() const
{
  return _crossingDetections;
}

void SegmentDetector::addStraightDetection(Segment::Ptr seg,
					   const Pose2D& pose)
{
  progressTTL();
  
  if (_straightDetections.size() > 0) {
    bool foundMatch = false;
    double positionDistance;
//     double angularDistance;
    double minPosDistance;
//     double minAngDistance;
    size_t minIdx = 0;
    
    for (size_t i = 0; i < _straightDetections.size(); i++) {
      positionDistance = _straightDetections[i].getPositionDistanceTo(pose);
//       angularDistance = _straightDetections[i].getAngularDistanceTo(pose);
      
      if (positionDistance < 0.6) {
	if (!foundMatch) {
	  minPosDistance = positionDistance;
// 	  minAngDistance = angularDistance;
	  minIdx = i;
	} else {
	  if (positionDistance < minPosDistance) {
	    minPosDistance = positionDistance;
// 	    minAngDistance = angularDistance;
	    minIdx = i;
	  }
	}
	foundMatch = true;
      }
    }
    
    if (foundMatch) {
      // Found matching detections, thus add new detection to this detection group
      _straightDetections[minIdx].add(pose);
    } else {
      // No matching straight detections, thus add a new detection group
      _straightDetections.push_back(StraightDetection(pose));
    }
  } else {
    // No previous straight detections, thus add a new detection group
    _straightDetections.push_back(StraightDetection(pose));
  }
  
  clearDeadDetections();
}

void SegmentDetector::addTCrossingLSDetection(Segment::Ptr seg,
					      const Pose2D& pose)
{
  addCrossingDetection(seg, pose, A2O::T_LS_CROSSING);
}

void SegmentDetector::addTCrossingLRDetection(Segment::Ptr seg,
					      const Pose2D& pose)
{
  addCrossingDetection(seg, pose, A2O::T_LR_CROSSING);
}

void SegmentDetector::addTCrossingSRDetection(Segment::Ptr seg,
					      const Pose2D& pose)
{
  addCrossingDetection(seg, pose, A2O::T_SR_CROSSING);
}

void SegmentDetector::addXCrossingDetection(Segment::Ptr seg,
					    const Pose2D& pose)
{
  addCrossingDetection(seg, pose, A2O::X_CROSSING);
}

void SegmentDetector::addCrossingDetection(Segment::Ptr seg,
					   const Pose2D& pose,
					   const CrossingType& type)
{
  progressTTL();
  
  if (_crossingDetections.size() > 0) {
    bool foundMatch = false;
    double positionDistance;
//     double angularDistance;
    double minPosDistance;
//     double minAngDistance;
    size_t minIdx = 0;
    
    for (size_t i = 0; i < _crossingDetections.size(); i++) {
      positionDistance = _crossingDetections[i].getPositionDistanceTo(pose);
//       angularDistance = _crossingDetections[i].getAngularDistanceTo(pose);
      
      if (positionDistance < 0.3) {
	if (!foundMatch) {
	  minPosDistance = positionDistance;
// 	  minAngDistance = angularDistance;
	  minIdx = i;
	} else {
	  if (positionDistance < minPosDistance) {
	    minPosDistance = positionDistance;
// 	    minAngDistance = angularDistance;
	    minIdx = i;
	  }
	}
	foundMatch = true;
      }
    }
    
    if (foundMatch) {
      // Found matching detections, thus add new detection to this detection group
      _crossingDetections[minIdx].add(pose, type);
    } else {
      // No matching straight detections, thus add a new detection group
      _crossingDetections.push_back(CrossingDetection(pose, type));
    }
  } else {
    // No previous straight detections, thus add a new detection group
    _crossingDetections.push_back(CrossingDetection(pose, type));
  }
  
  clearDeadDetections();
}

void SegmentDetector::addCurveDetection(Segment::Ptr seg,
					const Circle2D& circle,
					const bool& leftCurve)
{
  progressTTL();
  
  if (_curveDetections.size() > 0) {
    bool foundMatch = false;
    double positionDistance;
    double radiusDistance;
    double minPosDistance;
//     double minRadiusDistance;
    size_t minIdx = 0;
    
    for (size_t i = 0; i < _curveDetections.size(); i++) {
      if (_curveDetections[i].isLeftCurve() == leftCurve) {
	positionDistance = _curveDetections[i].getOriginDistanceTo(circle);
	radiusDistance = _curveDetections[i].getAbsRadiusDistanceTo(circle);
	
	if (positionDistance < 0.8 && radiusDistance < 1) {
	  if (!foundMatch) {
	    minPosDistance = positionDistance;
// 	    minRadiusDistance = radiusDistance;
	    minIdx = i;
	  } else {
	    if (positionDistance < minPosDistance) {
	      minPosDistance = positionDistance;
// 	      minRadiusDistance = radiusDistance;
	      minIdx = i;
	    }
	  }
	  foundMatch = true;
	}
      }
    }
    
    if (foundMatch) {
      // Found matching detections, thus add new detection to this detection group
      _curveDetections[minIdx].add(circle);
    } else {
      // No matching detections, thus add a new detection group
      _curveDetections.push_back(Curve2Detection(circle, leftCurve));
    }
  } else {
    // No previous detections, thus add a new detection group
    _curveDetections.push_back(Curve2Detection(circle, leftCurve));
  }
  
  clearDeadDetections();
}

void SegmentDetector::progressTTL()
{
  for (StraightDetection sd : _straightDetections) {
    sd.decrementTTL();
  }
  for (Curve2Detection cd : _curveDetections) {
    cd.decrementTTL();
  }
  for (CrossingDetection cd : _crossingDetections) {
    cd.decrementTTL();
  }
}

void SegmentDetector::clearDeadDetections()
{
  for (size_t i = 0; i < _straightDetections.size(); i++) {
    if (_straightDetections[i].getTTL() == 0) {
      _straightDetections.erase(_straightDetections.begin() + i);
      i--;
    }
  }
  for (size_t i = 0; i < _curveDetections.size(); i++) {
    if (_curveDetections[i].getTTL() == 0) {
      _curveDetections.erase(_curveDetections.begin() + i);
      i--;
    }
  }
  for (size_t i = 0; i < _crossingDetections.size(); i++) {
    if (_crossingDetections[i].getTTL() == 0) {
      _crossingDetections.erase(_crossingDetections.begin() + i);
      i--;
    }
  }
}

void SegmentDetector::reset()
{
  // Reset all remembered detections
  _straightDetections.clear();
  _curveDetections.clear();
  _crossingDetections.clear();
}

const bool SegmentDetector::redefineNextSegment(Segment::Ptr seg) const
{
  if (_straightDetections.size() + _curveDetections.size() + _crossingDetections.size() == 0) {
    return false;
  }
  
  size_t maxStraightIdx = 0;
  size_t maxCurveIdx = 0;
  size_t maxCrossingIdx = 0;
  
  size_t maxStraight = 0;
  size_t maxCurve = 0;
  size_t maxCrossing = 0;
  
  size_t currentCnt = 0;
  
  for (size_t i = 0; i < _straightDetections.size(); i++) {
    currentCnt = _straightDetections[i].getDetectionsCount();
    if (maxStraight == 0 || maxStraight < currentCnt) {
      maxStraight = currentCnt;
      maxStraightIdx = i;
    }
  }
  
  for (size_t i = 0; i < _curveDetections.size(); i++) {
    currentCnt = _curveDetections[i].getDetectionsCount();
    if (maxCurve == 0 || maxCurve < currentCnt) {
      maxCurve = currentCnt;
      maxCurveIdx = i;
    }
  }
  
  for (size_t i = 0; i < _crossingDetections.size(); i++) {
    currentCnt = _crossingDetections[i].getDetectionsCount();
    if (maxCrossing == 0 || maxCrossing < currentCnt) {
      maxCrossing = currentCnt;
      maxCrossingIdx = i;
    }
  }
  
  if (maxStraight > MIN_DETECTIONS_COUNT && maxStraight > maxCurve && maxStraight > maxCrossing) {
    return redefineStraight(seg, _straightDetections[maxStraightIdx]);
  } else if (maxCurve > MIN_DETECTIONS_COUNT && maxCurve > maxCrossing) {
    return redefineCurve(seg, _curveDetections[maxCurveIdx]);
  } else if (maxCrossing > MIN_DETECTIONS_COUNT) {
    return redefineCrossing(seg, _crossingDetections[maxCrossingIdx]);
  }
  
  return false;
}

const bool SegmentDetector::redefineStraight(Segment::Ptr seg,
					     const StraightDetection& straightDetection) const
{
  if (seg->isCrossing()) {
    // Do not modify anthing while on crossings
    return false;
  }
  
  Pose2D detectedPose = straightDetection.getPose();
  
  if (seg->isStraight()) {
    // We are on a straight and want to append a straight.
    // Thus delete next segment and extend current straight if the new detection is close to the end of the segment.
    SegmentLink::Ptr endLink = seg->getStraightOption();
    endLink->setSegmentAfter(Segment::Ptr());
    
    Pose2D relativePose = seg->getStraightOption()->getPose() / detectedPose;
    if (relativePose.x() > -1) {
      seg->setLength(seg->getLength() + 5);
      return true;
    }
  } else if (seg->isCurve()) {
    SegmentLink::Ptr endLink = seg->getStraightOption();
    endLink->setSegmentAfter(Segment::Ptr());
    
    double currentRadius = seg->getLength() / seg->getBendingAngle().rad();
    double exitAngleRad = seg->getBeginLink()->getPose().getAngle().getDifferenceTo(detectedPose.getAngle(), currentRadius < 0);
    Angle exitAngle = Angle::rad(exitAngleRad);
    double length = exitAngle.rad() * currentRadius;
    seg->set(length, exitAngle);
    
    Segment::appendSegment(endLink, 10);
    
    return true;
  }
  
  return false;
}

const bool SegmentDetector::redefineCurve(Segment::Ptr seg,
					  const Curve2Detection& curveDetection) const
{
  if (seg->isCrossing()) {
    // Do not modify anthing while on crossings
    return false;
  }
  
  Circle2D detectedCircle = curveDetection.getCircle();
  
  if (seg->isStraight()) {
    // Remove next segment
    SegmentLink::Ptr endLink = seg->getStraightOption();
    endLink->setSegmentAfter(Segment::Ptr());
    
    Vector2d localOrigin = seg->getBeginLink()->getPose() / detectedCircle.origin();
    seg->setLength(localOrigin(0));
    if (detectedCircle.radius() < 2) {
      if (curveDetection.isLeftCurve()) {
	Segment::appendCurve1MLeft(endLink);
      } else {
	Segment::appendCurve1MRight(endLink);
      }
    } else {
      if (curveDetection.isLeftCurve()) {
	Segment::appendCurve2MLeft(endLink);
      } else {
	Segment::appendCurve2MRight(endLink);
      }
    }
    
    return true;
  } else if (seg->isCurve()) {
    if (seg->isLeftCurve() != curveDetection.isLeftCurve()) {
      // We are in a left curve and detected a right curve
      // Remove next segment
      SegmentLink::Ptr endLink = seg->getStraightOption();
      endLink->setSegmentAfter(Segment::Ptr());
      
      double currentRadius = seg->getLength() / seg->getBendingAngle().rad();
      Vector2d localOrigin = seg->getBeginLink()->getPose() / detectedCircle.origin();
      Angle exitAngle = Angle::to(localOrigin - Vector2d(0, currentRadius));
      if (currentRadius > 0) {
	exitAngle += Angle::Deg_90();
      } else {
	exitAngle += Angle::Deg_N90();
      }
      
      double length = exitAngle.rad() * currentRadius;
      seg->set(length, exitAngle);
      
      if (detectedCircle.radius() < 2) {
	if (curveDetection.isLeftCurve()) {
	  Segment::appendCurve1MLeft(endLink);
	} else {
	  Segment::appendCurve1MRight(endLink);
	}
      } else {
	if (curveDetection.isLeftCurve()) {
	  Segment::appendCurve2MLeft(endLink);
	} else {
	  Segment::appendCurve2MRight(endLink);
	}
      }
      
      return true;
    }
  }
  
  return false;
}

const bool SegmentDetector::redefineCrossing(Segment::Ptr seg,
					     const CrossingDetection& crossingDetection) const
{
  if (seg->isCrossing()) {
    // Do not modify anthing while on crossings
    return false;
  }
  
  Pose2D detectedPose = crossingDetection.getPose();
  
  if (seg->isStraight()) {
    SegmentLink::Ptr endLink = seg->getStraightOption();
    endLink->setSegmentAfter(Segment::Ptr());
    
    Pose2D beginRelativePose = seg->getBeginLink()->getPose() / detectedPose;
    
    double length = beginRelativePose.x();
    seg->setLength(length);
    switch (crossingDetection.getMostLikelyType()) {
      case A2O::T_LS_CROSSING:
	Segment::appendTCrossingLS(endLink);
	break;
      case A2O::T_LR_CROSSING:
	Segment::appendTCrossingLR(endLink);
	break;
      case A2O::T_SR_CROSSING:
	Segment::appendTCrossingSR(endLink);
	break;
      case A2O::X_CROSSING:
      default:
	Segment::appendXCrossing(endLink);
	break;
    }
    
    return true;
  } else if (seg->isCurve()) {
    SegmentLink::Ptr endLink = seg->getStraightOption();
    endLink->setSegmentAfter(Segment::Ptr());
    
    double currentRadius = seg->getLength() / seg->getBendingAngle().rad();
    double exitAngleRad = seg->getBeginLink()->getPose().getAngle().getDifferenceTo(detectedPose.getAngle(), currentRadius < 0);
    Angle exitAngle = Angle::rad(exitAngleRad);
    double length = exitAngle.rad() * currentRadius;
    seg->set(length, exitAngle);
    
    switch (crossingDetection.getMostLikelyType()) {
      case A2O::T_LS_CROSSING:
	Segment::appendTCrossingLS(endLink);
	break;
      case A2O::T_LR_CROSSING:
	Segment::appendTCrossingLR(endLink);
	break;
      case A2O::T_SR_CROSSING:
	Segment::appendTCrossingSR(endLink);
	break;
      case A2O::X_CROSSING:
      default:
	Segment::appendXCrossing(endLink);
	break;
    }
    
    return true;
  }
  
  return false;
}
