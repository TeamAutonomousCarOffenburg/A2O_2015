#include "Axle.h"

using namespace A2O;
using namespace Eigen;

Axle::Axle(IAxleConfig::ConstPtr config)
      : _name(config->getName()),
        _position(config->getPosition()),
        _length(config->getLength())
{
  double halfLength = _length / 2;
  Angle angle = config->getWheelAngle();
  _leftWheel = boost::make_shared<Wheel>(config->getWheelDiameter(),
					 Pose2D(_position(0), halfLength, angle.negate()),
					 config->getLeftWheelTachoName());
  _rightWheel = boost::make_shared<Wheel>(config->getWheelDiameter(),
					 Pose2D(_position(0), -halfLength, angle),
					 config->getRightWheelTachoName());
}

Axle::~Axle()
{
}

const std::string& Axle::getName() const
{
  return _name;
}

const Eigen::Vector3d& Axle::getPosition() const
{
  return _position;
}

const double& Axle::getLength() const
{
  return _length;
}

IWheel::ConstPtr Axle::getLeftWheel() const
{
  return boost::dynamic_pointer_cast<const IWheel>(_leftWheel);
}

IWheel::ConstPtr Axle::getRightWheel() const
{
  return boost::dynamic_pointer_cast<const IWheel>(_rightWheel);
}
