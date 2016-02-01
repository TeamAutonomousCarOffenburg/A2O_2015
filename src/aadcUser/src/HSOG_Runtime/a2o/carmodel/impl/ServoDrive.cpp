#include "ServoDrive.h"


using namespace A2O;

ServoDrive::ServoDrive(IServoDriveConfig::ConstPtr config)
      : Actuator(config)
{

}

ServoDrive::~ServoDrive()
{
  
}

const double& ServoDrive::getPosition() const
{
  return _position; 
}

const double& ServoDrive::getPreviousPosition() const
{
  return _previousPosition;
}

const double& ServoDrive::getTargetPosition() const
{
  return _targetPosition;
}

void ServoDrive::setPosition(const double& position)
{
  _targetPosition = position;
  if (_lastMeasurementTime == 0) {
    // As long as we haven't receive any update,
    // we simply expect to have reached the target position.
    // This way, a servo without an attached sensor is also up to date
    _previousPosition = _position;
    _position = _targetPosition;
  }
}

const double& ServoDrive::getMinPosition() const
{
  return _minPosition;
}

const double& ServoDrive::getMaxPosition() const
{
  return _maxPosition;
}

const bool ServoDrive::update(IPerception::ConstPtr perception)
{
  IDoubleValuePerceptor::ConstPtr valuePerceptor = perception->getDoubleValuePerceptor(_perceptorName);
  
  if (valuePerceptor.get()) {
    _previousPosition = _position;
    _position = valuePerceptor->getValue();
    _lastMeasurementTime = valuePerceptor->getTime();
    return true;
  }
  
  return false;
}

const bool ServoDrive::reflectAction(IAction::Ptr action) const
{
  IDoubleValueEffector::Ptr valueEffector = action->getServoDriveEffector(_effectorName);
  
  if (valueEffector.get()) {
    valueEffector->setValue(_targetPosition, 0);
    return true;
  }
  
  return false;
}
