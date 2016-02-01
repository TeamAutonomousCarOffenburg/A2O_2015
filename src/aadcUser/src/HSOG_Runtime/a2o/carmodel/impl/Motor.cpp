#include "Motor.h"


using namespace A2O;

Motor::Motor(IActuatorConfig::ConstPtr config)
      : Actuator(config), _speed(0), _stopWatch(new Stopwatch())
{

}

Motor::~Motor()
{
  if (_stopWatch != nullptr) {
    delete _stopWatch;
    _stopWatch = nullptr;
  }
}

const double& Motor::getSpeed() const
{
  return _speed; 
}

void Motor::setSpeed(const double& speed)
{
  _speed = speed;
}

const bool Motor::update(IPerception::ConstPtr perception)
{
  if (_stopWatch != nullptr) {
    _stopWatch->start();

    if (_stopWatch->elapsedTimeMs() > 6000) {
      delete _stopWatch;
      _stopWatch = nullptr;
    }
  }

  return false;
}

const bool Motor::reflectAction(IAction::Ptr action) const
{
  IDoubleValueEffector::Ptr valueEffector = action->getMotorEffector(_effectorName);
  
  if (valueEffector.get()) {
    valueEffector->setValue(_speed, 0);
    return true;
  }
  
  return false;
}

const bool Motor::isInitialized() const
{
  return _stopWatch == nullptr;
}
