#include "CrossingDetection.h"


using namespace A2O;

CrossingDetection::CrossingDetection(const Pose2D& pose,
				     const CrossingType& type)
      : SegmentDetection(1), _pose(pose), _mostLikelyType(type), _TLSCount(0), _TLRCount(0), _TSRCount(0), _XCount(0)
{
  incrementType(type);
}

CrossingDetection::~CrossingDetection()
{
}

const Pose2D& CrossingDetection::getPose() const
{
  return _pose;
}

const CrossingType& CrossingDetection::getMostLikelyType() const
{
  return _mostLikelyType;
}

void CrossingDetection::add(const Pose2D& pose,
				  const CrossingType& type)
{
  _pose = Pose2D::average(_pose, pose, _detectionsCount, 1);
  incrementType(type);
  incrementDetectionsCount();
}

const double CrossingDetection::getPositionDistanceTo(const Pose2D& pose) const
{
  return _pose.getDistanceTo(pose);
}

const double CrossingDetection::getAngularDistanceTo(const Pose2D& pose) const
{
  return std::fabs(_pose.getAngle().getDistanceTo(pose.getAngle()));
}

void CrossingDetection::incrementType(const CrossingType& type)
{
  _TLSCount *= DISCOUNT;
  _TLRCount *= DISCOUNT;
  _TSRCount *= DISCOUNT;
  _XCount *= DISCOUNT;
  
  switch (type) {
    case A2O::T_LS_CROSSING:
      _TLSCount /= DISCOUNT;
      _TLSCount++;
      break;
    case A2O::T_LR_CROSSING:
      _TLRCount /= DISCOUNT;
      _TLRCount++;
      break;
    case A2O::T_SR_CROSSING:
      _TSRCount /= DISCOUNT;
      _TSRCount++;
      break;
    case A2O::X_CROSSING:
    default:
      _XCount /= DISCOUNT;
      _XCount++;
      break;
  }
  
  // Determine most probable crossing type
  if (_TLSCount > _TLRCount && _TLSCount > _TSRCount && _TLSCount > _XCount) {
    _mostLikelyType = A2O::T_LS_CROSSING;
  } else if (_TLRCount > _TSRCount && _TLRCount > _XCount) {
    _mostLikelyType = A2O::T_LR_CROSSING;
  } else if (_TSRCount > _XCount) {
    _mostLikelyType = A2O::T_SR_CROSSING;
  } else {
    _mostLikelyType = A2O::X_CROSSING;
  }
}
