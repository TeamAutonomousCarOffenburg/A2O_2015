#include "StraightDetection.h"


using namespace A2O;

StraightDetection::StraightDetection(const Pose2D& pose)
      : SegmentDetection(1), _pose(pose)
{
}

StraightDetection::~StraightDetection()
{
}

const Pose2D& StraightDetection::getPose() const
{
  return _pose;
}

void StraightDetection::add(const Pose2D& pose)
{
  _pose = Pose2D::average(_pose, pose, _detectionsCount, 1);
  incrementDetectionsCount();
}

const double StraightDetection::getPositionDistanceTo(const Pose2D& pose) const
{
  return _pose.getDistanceTo(pose);
}

const double StraightDetection::getAngularDistanceTo(const Pose2D& pose) const
{
  return std::fabs(_pose.getAngle().getDistanceTo(pose.getAngle()));
}
