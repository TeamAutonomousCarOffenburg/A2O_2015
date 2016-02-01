#include "SegmentLink.h"
#include "Segment.h"

using namespace A2O;


SegmentLink::SegmentLink()
{
}

SegmentLink::SegmentLink(const Pose2D& pose,
			 const Pose2D& localSupportPose,
			 boost::shared_ptr<Segment> segmentBefore)
      : _pose(pose), _localSupportPose(localSupportPose), _segmentBefore(segmentBefore)
{
}

SegmentLink::~SegmentLink()
{
}

void SegmentLink::setPose(const Pose2D& pose)
{
  _pose = pose;
  
  if (hasSegmentAfter()) {
    _segmentAfter->onBeginLinkUpdated();
  }
}

const Pose2D& SegmentLink::getPose() const
{
  return _pose;
}

const Pose2D& SegmentLink::getLocalSupportPose() const
{
  return _localSupportPose;
}

const Pose2D SegmentLink::getGlobalSupportPose() const
{
  return _pose * _localSupportPose;
}

int SegmentLink::getSupportPriority() const
{
  if (!_segmentBefore) {
    // Initial segment link --> maximum priority
    return -1;
  }

  if (_segmentBefore->getRightOption().get() == this) {
    return 0;
  } else if (_segmentBefore->getLeftOption().get() == this) {
    return 1;
  }

  return 100;
}

boost::shared_ptr<Segment> SegmentLink::getSegmentBefore() const
{
  return _segmentBefore;
}

boost::shared_ptr<Segment> SegmentLink::getSegmentAfter() const
{
  return _segmentAfter;
}

bool SegmentLink::hasSegmentBefore() const
{
  if (_segmentBefore) {
    return true;
  }
  
  return false;
}

bool SegmentLink::hasSegmentAfter() const
{
  if (_segmentAfter) {
    return true;
  }
  
  return false;
}

bool SegmentLink::isLeftOption() const
{
  if (_segmentBefore && _segmentBefore->getLeftOption().get() == this) {
    return true;
  }
  
  return false;
}

bool SegmentLink::isStraightOption() const
{
  if (_segmentBefore && _segmentBefore->getStraightOption().get() == this) {
    return true;
  }
  
  return false;
}

bool SegmentLink::isRightOption() const
{
  if (_segmentBefore && _segmentBefore->getRightOption().get() == this) {
    return true;
  }
  
  return false;
}

bool SegmentLink::isIntendedOption() const
{
  if (_segmentBefore && _segmentBefore->getIntendedOption().get() == this) {
    return true;
  }
  
  return false;
}

bool SegmentLink::hasPassedLink(const Pose2D& pose) const
{
  if (!_segmentBefore) {
    return true;
  } else if (!_segmentAfter) {
    return false;
  }

  Pose2D beginPose = _segmentBefore->getBeginLink()->getPose();

  // Construct the middle pose between the previous and this link, pointing to this pose
  Angle middleAngle = beginPose.getAngle() + beginPose.getAngleTo(_pose);
  Pose2D middleConnection((beginPose.x() + _pose.x()) / 2.0, (beginPose.y() + _pose.y()) / 2.0, middleAngle);

  Pose2D relativePose = middleConnection / pose;
  // If the pose is behind the middle connection pose, then it can't be passed this link
  if (relativePose.x() < 0) {
    return false;
  }

  // Check if the pose is in front of this link
  relativePose = _pose / pose;
  if (relativePose.x() > 0) {
    return true;
  }

  return false;
}

void SegmentLink::setSegmentAfter(boost::shared_ptr<Segment> segment)
{
  _segmentAfter = segment;
}
