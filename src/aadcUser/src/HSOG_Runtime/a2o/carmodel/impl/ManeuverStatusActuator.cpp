#include "ManeuverStatusActuator.h"
#include "action/IManeuverStatusEffector.h"

using namespace A2O;

ManeuverStatusActuator::ManeuverStatusActuator(IActuatorConfig::ConstPtr config)
      : Actuator(config)
{

}

ManeuverStatusActuator::~ManeuverStatusActuator()
{

}

const ManeuverStatus& ManeuverStatusActuator::getStatus() const
{
  return _status;
}

void ManeuverStatusActuator::setStatus(const ManeuverStatus& status)
{
  _status = status;
}

void ManeuverStatusActuator::setCurrentManeuver(const int& maneuverId)
{
	_currentManeuver = maneuverId;
}

const int& ManeuverStatusActuator::getCurrentManeuver() const
{
	return _currentManeuver;
}

const bool ManeuverStatusActuator::update(IPerception::ConstPtr perception)
{
  return false;
}

const bool ManeuverStatusActuator::reflectAction(IAction::Ptr action) const
{
  IManeuverStatusEffector::Ptr manEffector = action->getManeuverStatusEffector(_effectorName); 
  
  if (manEffector.get()) {
    manEffector->setStatus(static_cast<int>(_status));
    manEffector->setManeuverId(_currentManeuver);
    return true;
  }
  
  return false;
}
