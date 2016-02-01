#include "Sensor.h"
#include <boost/iterator/iterator_concepts.hpp>


using namespace A2O;
using namespace Eigen;

Sensor::Sensor(ISensorConfig::ConstPtr config)
      : Sensor(config->getName(), config->getPerceptorName(), config->getPose())
{

}

Sensor::Sensor(const std::string& name,
	       const std::string& perceptorName,
	       const Pose3D& pose)
      : _name(name), _perceptorName(perceptorName), _lastMeasurementTime(0), _pose(pose)
{
  
}

Sensor::~Sensor()
{
  
}

const std::string& Sensor::getName() const
{
  return _name;
}

const std::string& Sensor::getPerceptorName() const
{
  return _perceptorName;
}

const Pose3D& Sensor::getPose() const
{
  return _pose;
}

const long& Sensor::getLastMeasurementTime() const
{
  return _lastMeasurementTime;
}

const Vector3d Sensor::toCarSystem(const Vector3d& localPos) const
{
  return _pose * localPos;
}

const Quaterniond Sensor::toCarSystem(const Quaterniond& localOrientation) const
{
  return _pose.getOrientation() * localOrientation;
}

const Pose3D Sensor::toCarSystem(const Pose3D& localPose) const
{
  return _pose * localPose;
}

const bool Sensor::isInitialized() const
{
  return true;
}
