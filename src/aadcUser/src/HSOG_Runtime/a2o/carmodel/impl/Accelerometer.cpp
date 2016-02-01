#include "Accelerometer.h"
#include "perception/IAccelerometerPerceptor.h"


using namespace A2O;
using namespace Eigen;

Accelerometer::Accelerometer(ISensorConfig::ConstPtr config)
      : Sensor(config)
{

}

Accelerometer::~Accelerometer()
{
  
}

const Eigen::Vector3d& Accelerometer::getAcceleration() const
{
  return _acc;
}

const bool Accelerometer::update(IPerception::ConstPtr perception)
{
  IAccelerometerPerceptor::ConstPtr accPerceptor = perception->getAccelerometerPerceptor(_perceptorName);
  
  if (accPerceptor.get()) {
    _acc = accPerceptor->getAcceleration();
    _lastMeasurementTime = accPerceptor->getTime();
    return true;
  }
  
  return false;
}
