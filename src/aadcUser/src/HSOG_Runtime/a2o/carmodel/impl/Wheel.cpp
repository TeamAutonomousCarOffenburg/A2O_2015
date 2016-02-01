#include "Wheel.h"


using namespace A2O;
using namespace Eigen;

Wheel::Wheel(const double& diameter,
	     const Pose2D& pose,
	     std::string tachoName)
      : _diameter(diameter), _pose(pose), _tachoName(tachoName)
{
}

Wheel::~Wheel()
{
}

const double& Wheel::getDiameter() const
{
  return _diameter;
}

const Pose2D& Wheel::getPose() const
{
  return _pose;
}

const std::string& Wheel::getTachometerName() const
{
  return _tachoName;
}

const bool Wheel::hasTachometer() const
{
  return !_tachoName.empty();
}

const double Wheel::getRPS() const
{
  return 0;
}

const double Wheel::getSpeed() const
{
  return 0;
}
