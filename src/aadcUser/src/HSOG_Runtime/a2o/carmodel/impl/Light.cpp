#include "Light.h"


using namespace A2O;

Light::Light(IActuatorConfig::ConstPtr config)
      : Actuator(config)
{

}

Light::~Light()
{

}

const bool& Light::isOn() const
{
  return _active;
}

void Light::setState(const bool& active)
{
  _active = active;
}

void Light::turnOn()
{
  setState(true);
}

void Light::turnOff()
{
  setState(false);
}

const bool Light::update(IPerception::ConstPtr perception)
{
  return false;
}

const bool Light::reflectAction(IAction::Ptr action) const
{
  IBoolValueEffector::Ptr boolEffector = action->getLightEffector(_effectorName);
  
  if (boolEffector.get()) {
    boolEffector->setValue(_active, 0);
    return true;
  }
  
  return false;
}
