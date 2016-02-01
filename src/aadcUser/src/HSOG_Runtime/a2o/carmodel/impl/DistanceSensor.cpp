#include "DistanceSensor.h"

#include <cmath>


using namespace A2O;

DistanceSensor::DistanceSensor(IDistanceSensorConfig::ConstPtr config)
      : Sensor(config), _minDistance(config->getMinDistance()), _maxDistance(config->getMaxDistance()), _scanWidth(config->getScanWidth())
{
}

DistanceSensor::~DistanceSensor()
{
}

const double& DistanceSensor::getMinDistance() const
{
  return _minDistance;
}

const double& DistanceSensor::getMaxDistance() const
{
  return _maxDistance;
}

const double& DistanceSensor::getScanWidth() const
{
  return _scanWidth;
}

const bool DistanceSensor::inRange() const
{
  return _distance >= _minDistance && _distance <= _maxDistance;
}

const double DistanceSensor::getPercent() const
{
  double diff = _distance - _minDistance;
  return diff / (_maxDistance - _minDistance);
}

/**
 * \brief returns the currently cached distance
 * 
 * \return double: distance in meter
 */

const double& DistanceSensor::getDistance() const
{
  std::lock_guard<std::mutex> guard(_mutex);
  return _distance;
}

const double& DistanceSensor::getLimitedDistance() const
{
  std::lock_guard<std::mutex> guard(_mutex);
  return std::min(std::max(_distance, _minDistance), _maxDistance);
}

const bool DistanceSensor::update(const A2O::IPerception::ConstPtr perception)
{
  IDoubleValuePerceptor::ConstPtr valuePerceptor = perception->getDoubleValuePerceptor(_perceptorName);
  
  if (valuePerceptor.get()) {
    {
      std::lock_guard<std::mutex> guard(_mutex);
      _distance = valuePerceptor->getValue();
      _lastMeasurementTime = valuePerceptor->getTime();
    }
    return true;
  }
  
  return false;
}
