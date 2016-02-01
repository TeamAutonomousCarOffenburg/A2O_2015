#include "Segment.h"
#include <worldmodel/IDriveInstructionConstants.h>

#include "utils/geometry/Geometry.h"
#include <cmath>

using namespace A2O;

Pose2D Segment::LEFT_EXIT_POSE(0.5, 0.5, Angle::Deg_90());
Pose2D Segment::RIGHT_EXIT_POSE(0.5, -0.5, Angle::Deg_N90());


Segment::Segment(const double& length,
		 const Angle& bendingAngle)
      : _length(length), _bendingAngle(bendingAngle), _consumesDriveInstruction(false)
{
}

Segment::~Segment()
{
}

void Segment::setBendingAngle(const Angle& bendingAngle)
{
  _bendingAngle = bendingAngle;
  
  if (_straightOption) {
    Pose2D localEndPose(Geometry::calculateArcPose(_length, _bendingAngle));
    _straightOption->setPose(_begin->getPose() * localEndPose);
  }
}

const Angle& Segment::getBendingAngle() const
{
  return _bendingAngle;
}

void Segment::setLength(const double& length)
{
  if (length < 0) {
    _length = 0;
  } else {
    _length = length;
  }
  
  if (_straightOption) {
    Pose2D localEndPose(Geometry::calculateArcPose(_length, _bendingAngle));
    _straightOption->setPose(_begin->getPose() * localEndPose);
  }
}

const double& Segment::getLength() const
{
  return _length;
}

void Segment::set(const double& length, const Angle& exitAngle)
{
  if (length < 0) {
    _length = 0;
  } else {
    _length = length;
  }
  _bendingAngle = exitAngle;
  
  if (_straightOption) {
    Pose2D localEndPose(Geometry::calculateArcPose(_length, _bendingAngle));
    _straightOption->setPose(_begin->getPose() * localEndPose);
  }
}

SegmentLink::Ptr Segment::getBeginLink() const
{
  return _begin;
}

SegmentLink::Ptr Segment::getLeftOption() const
{
  return _leftOption;
}

SegmentLink::Ptr Segment::getStraightOption() const
{
  return _straightOption;
}

SegmentLink::Ptr Segment::getRightOption() const
{
  return _rightOption;
}

SegmentLink::Ptr Segment::getIntendedOption() const
{
  return _intendedOption;
}

const bool& Segment::consumesDriveInstruction() const
{
  return _consumesDriveInstruction;
}

const std::vector<RoadSign::Ptr>& Segment::getRoadSigns() const
{
  return _roadSigns;
}

void Segment::attachRoadSigns(const std::vector<RoadSign::Ptr>& roadSigns)
{
  _roadSigns = roadSigns;
}

bool Segment::hasLeftOption() const
{
  return _leftOption.get() != nullptr;
}

bool Segment::hasStraightOption() const
{
  return _straightOption.get() != nullptr;
}

bool Segment::hasRightOption() const
{
  return _rightOption.get() != nullptr;
}

bool Segment::hasIntendedOption() const
{
  return _intendedOption.get() != nullptr;
}

bool Segment::isStraight() const
{
  return std::fabs(_bendingAngle.rad()) < 0.001;
}

bool Segment::isCurve() const
{
  return !isStraight();
}

bool Segment::isLeftCurve() const
{
  return isCurve() && _bendingAngle.rad() >= 0;
}

bool Segment::isRightCurve() const
{
  return isCurve() && _bendingAngle.rad() < 0;
}

bool Segment::isCrossing() const
{
  int exitOptions = 0;

  if (_leftOption) {
    exitOptions++;
  }
  if (_straightOption) {
    exitOptions++;
  }
  if (_rightOption) {
    exitOptions++;
  }

  return exitOptions > 1;
}

bool Segment::isXCrossing() const
{
  return _leftOption.get() != nullptr && _straightOption.get() != nullptr && _rightOption.get() != nullptr;
}

bool Segment::isTCrossingLS() const
{
  return _leftOption.get() != nullptr && _straightOption.get() != nullptr && _rightOption.get() == nullptr;
}

bool Segment::isTCrossingLR() const
{
  return _leftOption.get() != nullptr && _straightOption.get() == nullptr && _rightOption.get() != nullptr;
}

bool Segment::isTCrossingSR() const
{
  return _leftOption.get() == nullptr && _straightOption.get() != nullptr && _rightOption.get() != nullptr;
}

bool Segment::isLeftOptionAllowed() const
{
  if (_leftOption) {
    for (RoadSign::Ptr sign : _roadSigns) {
      switch (sign->getSignType()) {
	case A2O::ForceStraight:
	  return false;
	default:
	  break;
      }
    }
    
    return true;
  }
  
  return false;
}

bool Segment::isStraightOptionAllowed() const
{
  if (_straightOption) {
    // Currently no sign prevents us from taking this option
//     for (RoadSign::Ptr sign : _roadSigns) {
//       switch (sign->getSignType()) {
// 	case ??:
// 	  return false;
// 	default:
// 	  break;
//       }
//     }

    return true;
  }

  return false;
}

bool Segment::isRightOptionAllowed() const
{
  if (_rightOption) {
    for (RoadSign::Ptr sign : _roadSigns) {
      switch (sign->getSignType()) {
	case A2O::ForceStraight:
	  return false;
	default:
	  break;
      }
    }
    
    return true;
  }
  
  return false;
}

void Segment::appendIntermediateWayPoints(const int& howMany,
					  std::vector<Pose2D>& wayPoints) const
{
  if (howMany <= 0) {
    return;
  }
  
  if (_bendingAngle.rad() == 0) {
    double extend = _length / (howMany + 1);
    for (int i = 1; i <= howMany; i++) {
      wayPoints.push_back(_begin->getPose() * Pose2D(i * extend, -0.25));
    }
  } else {
    double radius = (_length / _bendingAngle.rad()) + 0.25;
    double extend = _bendingAngle.rad() * radius / (howMany + 1);
    double radExtend = _bendingAngle.rad() / (howMany + 1);
    for (int i = 1; i <= howMany; i++) {
      wayPoints.push_back(_begin->getPose() * Pose2D(0, -0.25) * Geometry::calculateArcPose(i * extend, Angle::rad(i * radExtend)));
    }
  }
}

void Segment::appendIntermediateWayPoints(const int& howMany,
					  Path2D::Ptr path) const
{
  if (howMany <= 0) {
    return;
  }
  
  if (_bendingAngle.rad() == 0) {
    double extend = _length / (howMany + 1);
    for (int i = 1; i <= howMany; i++) {
      path->addWayPoint(_begin->getPose() * Pose2D(i * extend, -0.25));
    }
  } else {
    double radius = (_length / _bendingAngle.rad()) + 0.25;
    double extend = _bendingAngle.rad() * radius / (howMany + 1);
    double radExtend = _bendingAngle.rad() / (howMany + 1);
    for (int i = 1; i <= howMany; i++) {
      path->addWayPoint(_begin->getPose() * Pose2D(0, -0.25) * Geometry::calculateArcPose(i * extend, Angle::rad(i * radExtend)));
    }
  }
}

SegmentLink::Ptr Segment::getClosestLinkToPose(const Pose2D& pose) const
{
    Angle bestAngle = Angle::deg(179);
    SegmentLink::Ptr bestOption = getLeftOption();
    
    Angle poseAngle = pose.getAngle();
    Angle diffAngle;
    if(hasRightOption())
    {
     Angle right = getRightOption()->getPose().getAngle();
     diffAngle = poseAngle-right; 
    
     if(fabs(diffAngle.rad()) < fabs(bestAngle.rad()))
     {
       bestAngle = diffAngle;
       bestOption = getRightOption();
     }
    }
  
     if(hasStraightOption())
     {
     Angle straight = getStraightOption()->getPose().getAngle();
     diffAngle = poseAngle - straight;
     if(fabs(diffAngle.rad()) < fabs(bestAngle.rad()))
     {
       bestOption = getStraightOption();
     }
     }
     
     return bestOption;
}

Segment::Ptr Segment::getNextOrCurrentSegmentFromPose(Segment::Ptr currentSegment, const Pose2D& pose)
{
 // set beginLink as origin
 Pose2D beginLink = currentSegment->getBeginLink()->getPose();

 Pose2D poseLocalToBegin =  beginLink / pose;
 double length = currentSegment->getLength();
 
 if(!currentSegment->isCrossing())
 {
    Pose2D endLink = currentSegment->getStraightOption()->getPose();
    
    // get the middlePoint between begin and end and set coordinate system, so its easy to use
    Angle middleAngle = beginLink.getAngle() + beginLink.getAngleTo(endLink);
    Pose2D middleConnection((beginLink.x() + endLink.x()) / 2.0, (beginLink.y() + endLink.y()) / 2.0, middleAngle);
    
    Pose2D poseRelativeToMiddleConnection = middleConnection / pose;
    // these are guaranteed to be on this segment
    if(poseRelativeToMiddleConnection.x() < 0)
    {
      return currentSegment;
    }
    
    Pose2D poseRelativeToEnd = endLink / pose;
    if(poseRelativeToEnd.x() < 0)
    {
      return currentSegment;
    }
    
    return currentSegment->getStraightOption()->getSegmentAfter();
  }
   
 
 // check if in current
 if(  poseLocalToBegin.x() >= 0  &&  poseLocalToBegin.x() <= length &&
    poseLocalToBegin.y() >= -(length / 2.0) && poseLocalToBegin.y() <= (length / 2.0))
 {
    return currentSegment;
 }
 
 // get middle and rotate 45 degrees, so it is easier to decide where the pose is
 Pose2D middleOfCrossingLocal = Pose2D(length/2, 0, Angle::deg(45));
 Pose2D poseRelativeToMiddle =  (middleOfCrossingLocal * beginLink) / pose;
 
 
 Angle toPose = poseRelativeToMiddle.getAngle();

 double deg = toPose.deg();
 if(currentSegment->hasLeftOption() && deg >= 0 && deg < 90)
 {
   return currentSegment->getLeftOption()->getSegmentAfter();
 }
 else if(deg >= 90 && deg <= 180)
 {
    return currentSegment;
 }
 else if(currentSegment->hasStraightOption() && deg < 0 && deg > -90)
 {
   return currentSegment->getStraightOption()->getSegmentAfter();
 }
 else if(currentSegment->hasRightOption() && deg <= -90 && deg >= -180)
 {
   return currentSegment->getRightOption()->getSegmentAfter();
 }
 return currentSegment;
 
}

void Segment::update(DriveInstructionManager::Ptr driveInstructionManager,
		     size_t instructionIndex)
{
  if (isCrossing()) {
    std::string driveInstruction = driveInstructionManager->getInstruction(instructionIndex);

    if ((driveInstruction == DRIVE_INSTRUCTION::DRIVE_LEFT || driveInstruction == DRIVE_INSTRUCTION::PULL_OUT_LEFT)
	  && isLeftOptionAllowed()) {
      _intendedOption = _leftOption;
      _consumesDriveInstruction = true;
    } else if (driveInstruction == DRIVE_INSTRUCTION::DRIVE_STRAIGHT && isStraightOptionAllowed()) {
      _intendedOption = _straightOption;
      _consumesDriveInstruction = true;
    } else if ((driveInstruction == DRIVE_INSTRUCTION::DRIVE_RIGHT || driveInstruction == DRIVE_INSTRUCTION::PULL_OUT_RIGHT)
	  && isRightOptionAllowed()) {
      _intendedOption = _rightOption;
      _consumesDriveInstruction = true;
    } else {
      // We have no matching instruction for this crossing, decide for the first best option
      if (_straightOption) {
	_intendedOption = _straightOption;
      } else if (_leftOption) {
	_intendedOption = _leftOption;
      } else {
	_intendedOption = _rightOption;
      }
      _consumesDriveInstruction = false;
    }
  } else {
    _intendedOption = _straightOption;
    _consumesDriveInstruction = false;
  }

  // Progress instruction index
  if (_consumesDriveInstruction) {
    instructionIndex++;
  }
  
  // Update subsequent segment if present
  if (_intendedOption->hasSegmentAfter() && instructionIndex < driveInstructionManager->getNumberOfInstructions() + 5) {
    _intendedOption->getSegmentAfter()->update(driveInstructionManager, instructionIndex);
  }
}

void Segment::onBeginLinkUpdated()
{
  if (_straightOption) {
    Pose2D localEndPose(Geometry::calculateArcPose(_length, _bendingAngle));
    _straightOption->setPose(_begin->getPose() * localEndPose);
  }
  
  if (_leftOption) {
    _leftOption->setPose(_begin->getPose() * LEFT_EXIT_POSE);
  }
  
  if (_rightOption) {
    _rightOption->setPose(_begin->getPose() * RIGHT_EXIT_POSE);
  }
}

// void Segment::addStraightDetection()
// {
//   _straightDetections++;
// }
// 
// void Segment::addLeftCurveDetection(const Circle2D& circle)
// {
//   addDetection(_leftCurveDetections, circle);
// }
// 
// void Segment::addRightCurveDetection(const Circle2D& circle)
// {
//   addDetection(_rightCurveDetections, circle);
// }
// 
// void Segment::addTLSDetection(const Pose2D& pose)
// {
//   addDetection(_TLSDetections, pose);
// }
// 
// void Segment::addTLRDetection(const Pose2D& pose)
// {
//   addDetection(_TLRDetections, pose);
// }
// 
// void Segment::addTSRDetection(const Pose2D& pose)
// {
//   addDetection(_TSRDetections, pose);
// }
// 
// void Segment::addXDetection(const Pose2D& pose)
// {
//   addDetection(_XDetections, pose);
// }
// 
// void Segment::addDetection(std::pair<int, Circle2D>& detections,
// 			   const Circle2D& newDetection)
// {
//   Circle2D avgCircle;
//   if (detections.first <= 0) {
//     avgCircle = newDetection;
//   } else {
//     avgCircle = Circle2D::avg(detections.second, newDetection, detections.first, 1);
//   }
//   
//   detections.first++;
//   detections.second = avgCircle;
// }
// 
// void Segment::addDetection(std::pair<int, Pose2D>& detections,
// 			   const Pose2D& newDetection)
// {
//   Pose2D avgPose;
//   if (detections.first <= 0) {
//     avgPose = newDetection;
//   } else {
//     avgPose = Pose2D::average(detections.second, newDetection, detections.first, 1);
//   }
//   
//   detections.first++;
//   detections.second = avgPose;
// }
// 
// void Segment::redefineSegmentBasedOnDetections(Segment::Ptr seg)
// {
//   seg->_intendedOption = SegmentLink::Ptr();
//   
//   int maxCurve = std::max(seg->_leftCurveDetections.first, seg->_rightCurveDetections.first);
//   int maxCrossing = seg->_TLSDetections.first + seg->_TLRDetections.first + seg->_TSRDetections.first + seg->_XDetections.first;
//   
//   if (seg->_straightDetections > maxCrossing && seg->_straightDetections > maxCrossing) {
//     // We should become a straight segment
//     seg->_leftOption = SegmentLink::Ptr();
//     seg->_rightOption = SegmentLink::Ptr();
//     
//     seg->setBendingAngle(0);
//     
//     if (!seg->_straightOption) {
//       Pose2D localEndPose(Geometry::calculateArcPose(seg->_length, seg->_bendingAngle));
//       seg->_straightOption = boost::make_shared<SegmentLink>(seg->_begin->getPose() * localEndPose, Pose2D(0, -0.25), seg);
//     }
//   } else if (maxCurve > maxCrossing) {
//     // We should become a curved segment
//     seg->_leftOption = SegmentLink::Ptr();
//     seg->_rightOption = SegmentLink::Ptr();
//     if (!seg->_straightOption) {
//       // TODO: Redefine length and bending angle based on extracted curve
//       Pose2D localEndPose(Geometry::calculateArcPose(seg->_length, seg->_bendingAngle));
//       seg->_straightOption = boost::make_shared<SegmentLink>(seg->_begin->getPose() * localEndPose, Pose2D(0, -0.25), seg);
//     }
//   } else {
//     // We should become a crossing segment
//     // Thus average all crossing begin poses
//     int weight = 0;
//     Pose2D avgPose = Pose2D(0, 0);
//     if (seg->_TLSDetections.first > 0) {
//       avgPose = seg->_TLSDetections.second;
//       weight = seg->_TLSDetections.first;
//     }
//     
//     if (seg->_TLRDetections.first > 0) {
//       if (weight == 0) {
// 	avgPose = seg->_TLRDetections.second;
// 	weight = seg->_TLRDetections.first;
//       } else {
// 	avgPose = Pose2D::average(avgPose, seg->_TLRDetections.second, weight, seg->_TLRDetections.first);
// 	weight += seg->_TLRDetections.first;
//       }
//     }
//     
//     if (seg->_TSRDetections.first > 0) {
//       if (weight == 0) {
// 	avgPose = seg->_TSRDetections.second;
// 	weight = seg->_TSRDetections.first;
//       } else {
// 	avgPose = Pose2D::average(avgPose, seg->_TSRDetections.second, weight, seg->_TSRDetections.first);
// 	weight += seg->_TSRDetections.first;
//       }
//     }
//     
//     if (seg->_XDetections.first > 0) {
//       if (weight == 0) {
// 	avgPose = seg->_XDetections.second;
// 	weight = seg->_XDetections.first;
//       } else {
// 	avgPose = Pose2D::average(avgPose, seg->_XDetections.second, weight, seg->_XDetections.first);
// 	weight += seg->_XDetections.first;
//       }
//     }
//     
//     
//     if (seg->_TLSDetections.first > seg->_TLRDetections.first
// 	&& seg->_TLSDetections.first > seg->_TSRDetections.first
// 	&& seg->_TLSDetections.first > seg->_XDetections.first) {
//       // We should become a T-LS segment
//       
//     } else if (seg->_TLRDetections.first > seg->_TSRDetections.first
// 	&& seg->_TLRDetections.first > seg->_XDetections.first) {
//       // We should become a T-LR segment
//       
//     } else if (seg->_TSRDetections.first > seg->_XDetections.first) {
//       // We should become a T-SR segment
//       
//     } else {
//       // We should become a X segment
//     }
//   }
// }





// ===== Static create methods ==========
Segment::Ptr Segment::createInitialTCrossingLR(const Pose2D& initialPose,
					       const Pose2D& initialSupportPose)
{
  SegmentLink::Ptr initialLink(boost::make_shared<SegmentLink>(initialPose, initialSupportPose, Segment::Ptr()));

  return appendTCrossingLR(initialLink);
}

Segment::Ptr Segment::createInitialSegment(const Pose2D& initialPose,
					   const double& length,
					   const Angle& bendingAngle)
{
  SegmentLink::Ptr initialLink(boost::make_shared<SegmentLink>(initialPose, Pose2D(0, -0.25), Segment::Ptr()));
  
  return appendSegmentInternal(initialLink, length, bendingAngle);
}

Segment::Ptr Segment::appendSegment(SegmentLink::Ptr fromLink,
				    const double& length,
				    const Angle& bendingAngle)
{ 
  return appendSegmentInternal(fromLink, length, bendingAngle);
}

Segment::Ptr Segment::appendXCrossing(SegmentLink::Ptr fromLink)
{
  Segment::Ptr newSegment = appendSegmentInternal(fromLink, 1);
  
  // Create Left/Right options
  newSegment->_leftOption = boost::make_shared<SegmentLink>(newSegment->_begin->getPose() * LEFT_EXIT_POSE, Pose2D(0, -0.25), newSegment);
  newSegment->_rightOption = boost::make_shared<SegmentLink>(newSegment->_begin->getPose() * RIGHT_EXIT_POSE, Pose2D(0.15, -0.15), newSegment);
  
  // Add a 10 meter straight segment to each option
  appendSegmentInternal(newSegment->_leftOption, 10);
  appendSegmentInternal(newSegment->_straightOption, 10);
  appendSegmentInternal(newSegment->_rightOption, 10);
  
  return newSegment;
}

Segment::Ptr Segment::appendTCrossingLS(SegmentLink::Ptr fromLink)
{
  Segment::Ptr newSegment = appendSegmentInternal(fromLink, 1);
  
  // Create Left option
  newSegment->_leftOption = boost::make_shared<SegmentLink>(newSegment->_begin->getPose() * LEFT_EXIT_POSE, Pose2D(0, -0.25), newSegment);
  
  // Add a 10 meter straight segment to each option
  appendSegmentInternal(newSegment->_leftOption, 10);
  appendSegmentInternal(newSegment->_straightOption, 10);
  
  return newSegment;
}

Segment::Ptr Segment::appendTCrossingLR(SegmentLink::Ptr fromLink)
{
  Segment::Ptr newSegment = appendSegmentInternal(fromLink, 1, Angle::Zero(), false);
  
  // Create Left and Right options
  newSegment->_leftOption = boost::make_shared<SegmentLink>(newSegment->_begin->getPose() * LEFT_EXIT_POSE, Pose2D(0, -0.25), newSegment);
  newSegment->_rightOption = boost::make_shared<SegmentLink>(newSegment->_begin->getPose() * RIGHT_EXIT_POSE, Pose2D(0.15, -0.15), newSegment);
  
  // Add a 10 meter straight segment to each option
  appendSegmentInternal(newSegment->_leftOption, 10);
  appendSegmentInternal(newSegment->_rightOption, 10);
  
  return newSegment;
}

Segment::Ptr Segment::appendTCrossingSR(SegmentLink::Ptr fromLink)
{
  Segment::Ptr newSegment = appendSegmentInternal(fromLink, 1);
  
  // Create Right options
  newSegment->_rightOption = boost::make_shared<SegmentLink>(newSegment->_begin->getPose() * RIGHT_EXIT_POSE, Pose2D(0.15, -0.15), newSegment);
  
  // Add a 10 meter straight segment to each option
  appendSegmentInternal(newSegment->_straightOption, 10);
  appendSegmentInternal(newSegment->_rightOption, 10);
  
  return newSegment;
}

Segment::Ptr Segment::appendCurve1MLeft(SegmentLink::Ptr fromLink)
{
  return appendSegmentInternal(fromLink, 3 * M_PI / 4, Angle::Deg_90());
}

Segment::Ptr Segment::appendCurve1MRight(SegmentLink::Ptr fromLink)
{
  return appendSegmentInternal(fromLink, 3 * M_PI / 4, Angle::Deg_N90());
}

Segment::Ptr Segment::appendCurve2MLeft(SegmentLink::Ptr fromLink)
{
  return appendSegmentInternal(fromLink, 5 * M_PI / 4, Angle::Deg_90());
}

Segment::Ptr Segment::appendCurve2MRight(SegmentLink::Ptr fromLink)
{
  return appendSegmentInternal(fromLink, 5 * M_PI / 4, Angle::Deg_N90());
}

Segment::Ptr Segment::appendSegmentInternal(SegmentLink::Ptr fromLink,
					    const double& length,
					    const Angle& bendingAngle,
					    bool createStraightOption)
{
//   if (!fromLink) {
//     // There is no initial link available, thus create one for this track segment
//     fromLink = boost::make_shared<SegmentLink>();
//   }
  
  // Create a new segment
  Segment::Ptr newSegment(boost::make_shared<Segment>(length, bendingAngle));
  
  // Wire up new segment with preceding one
  newSegment->_begin = fromLink;
  newSegment->_begin->setSegmentAfter(newSegment);
  
  if (createStraightOption) {
    Pose2D localEndPose(Geometry::calculateArcPose(length, bendingAngle));
    newSegment->_straightOption = boost::make_shared<SegmentLink>(fromLink->getPose() * localEndPose, Pose2D(0, -0.25), newSegment);
  }
  
  return newSegment;
}
