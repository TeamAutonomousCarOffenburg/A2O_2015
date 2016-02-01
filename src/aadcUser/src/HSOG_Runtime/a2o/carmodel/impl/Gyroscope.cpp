#include "Gyroscope.h"

#include "utils/geometry/Geometry.h"
#include "perception/IGyroPerceptor.h"


using namespace A2O;
using namespace Eigen;

Gyroscope::Gyroscope(ISensorConfig::ConstPtr config)
      : Sensor(config), _gyro(), _orientation(), _initialOrientation(), _distribution(new Distribution(40))
{
  _gyro.setIdentity();
  _orientation.setIdentity();
  _initialOrientation.setIdentity();
}

Gyroscope::~Gyroscope()
{
  if (_distribution != nullptr) {
    delete _distribution;
    _distribution = nullptr;
  }
}

const Eigen::Quaterniond& Gyroscope::getGyro() const
{
  return _orientation; 
}

const Angle Gyroscope::getHorizontalAngle() const
{
  return Geometry::getHorizontalAngle(_orientation);
}

const bool Gyroscope::update(IPerception::ConstPtr perception)
{
  IGyroPerceptor::ConstPtr gyroPerceptor = perception->getGyroPerceptor(_perceptorName);
  
  if (gyroPerceptor.get()) {
    _gyro = gyroPerceptor->getGyro();
    _orientation = _initialOrientation * _gyro;
    _lastMeasurementTime = gyroPerceptor->getTime();

    // Check if the gyro sensor is initialized
    if (_distribution != nullptr) {
      _distribution->addValue(Geometry::getHorizontalAngle(_gyro).deg());
      if (_distribution->isValid() && _distribution->getVarianz() < 0.05) {
	    reset();
	    delete _distribution;
        _distribution = nullptr;
      }
    }

    return true;
  }
  
  return false;
}

const bool Gyroscope::isInitialized() const
{
  return _distribution == nullptr;
}

void Gyroscope::reset()
{
  _initialOrientation = _gyro.inverse();
}
