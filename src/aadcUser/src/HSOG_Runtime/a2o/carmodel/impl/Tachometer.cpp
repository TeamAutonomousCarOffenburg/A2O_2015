#include "Tachometer.h"
#include "perception/IWheelTickPerceptor.h"


using namespace A2O;


Tachometer::Tachometer(ISensorConfig::ConstPtr config)
      : Sensor(config)
{

}

Tachometer::~Tachometer()
{
  
}

const double& Tachometer::getRPM() const
{
  return _rpm; 
}

const bool Tachometer::update(IPerception::ConstPtr perception)
{
  IWheelTickPerceptor::ConstPtr tickPerceptor = perception->getWheelTickPerceptor(_perceptorName);

  if (tickPerceptor.get()) {
    _rpm = tickPerceptor->getTicks();
    _lastMeasurementTime = tickPerceptor->getTime();
    return true;
  }
  return false;
}
