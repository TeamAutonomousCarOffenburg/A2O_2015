#include "RoadSign.h"


using namespace A2O;

RoadSign::RoadSign(const SignType& type,
		   const Pose2D& pose)
      : VisibleObject(getNameForSign(type), pose), _sign(type)
{
}

RoadSign::~RoadSign()
{
}

const SignType& RoadSign::getSignType() const
{
  return _sign;
}

void RoadSign::update(const Pose2D& pose)
{
  _pose = Pose2D::average(_pose, pose, _seenCount, 1);
  _seenCount++;
}

int RoadSign::getSeenCount() const
{
	return _seenCount;
}

// ========== Static helper methods ====================
const std::string RoadSign::getNameForSign(const SignType& type)
{
  switch (type) {
    case A2O::Crossing:
      return "Crossing";
    case A2O::ForceStraight:
      return "ForceStraight";
    case A2O::GiveWay:
      return "GiveWay";
    case A2O::NoOvertaking:
      return "NoOvertaking";
    case A2O::OneWayStreet:
      return "OneWayStreet";
    case A2O::Parking:
      return "Parking";
    case A2O::PedestrianCrossing:
      return "PedestrianCrossing";
    case A2O::Roundabout:
      return "Roundabout";
    case A2O::Stop:
      return "Stop";
    case A2O::PriorityLane:
      return "PriorityLane";
    case A2O::NoEntry:
      return "NoEntry";
    default:
      return "NoMatch";
  }
}
