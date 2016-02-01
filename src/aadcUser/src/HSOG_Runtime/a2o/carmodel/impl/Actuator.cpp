#include "Actuator.h"


using namespace A2O;

Actuator::Actuator(IActuatorConfig::ConstPtr config)
      : Actuator(config->getName(), config->getPerceptorName(), config->getEffectorName())
{

}

Actuator::Actuator(const std::string& name,
		   const std::string& perceptorName,
		   const std::string& effectorName)
      : _name(name), _perceptorName(perceptorName), _effectorName(effectorName), _lastMeasurementTime(0)
{
  
}

Actuator::~Actuator()
{
  
}

const std::string& Actuator::getName() const
{
  return _name;
}

const std::string& Actuator::getPerceptorName() const
{
  return _perceptorName;
}

const std::string& Actuator::getEffectorName() const
{
  return _effectorName;
}

const long& Actuator::getLastMeasurementTime() const
{
  return _lastMeasurementTime;
}

const bool Actuator::isInitialized() const
{
  return true;
}
