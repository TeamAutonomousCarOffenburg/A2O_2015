#include "GyroOdometry.h"

#include <cmath>
#include "utils/geometry/Geometry.h"

#define CORRECTION_FACTOR 0.87
#define MIN_RADIUS 0.3

using namespace A2O;

GyroOdometry::GyroOdometry(const double& wheelDiameter,
			   const int& ticksPerRound)
      : _wheelDiameter(wheelDiameter), _ticksPerRound(ticksPerRound), _pose(), _previousTicks(0), _previousAngle(0)
{

}

GyroOdometry::~GyroOdometry()
{

}

const Pose2D& GyroOdometry::getPose() const
{
  return _pose;
}

void GyroOdometry::init(const double& ticks,
			const Angle& zAngle)
{
  _previousTicks = ticks;
  _previousAngle = zAngle;
  _pose = Pose2D(0, 0);
}

void GyroOdometry::reset(const Pose2D& pose)
{
//   Angle diffAngle = pose.getAngle() - _pose.getAngle();
//   _previousAngle += diffAngle;
  _pose = pose;
}

const Pose2D& GyroOdometry::update(const double& ticks,
				   const Angle& zAngle)
{
  double newWheelTicks = ticks - _previousTicks;

  if (std::fabs(newWheelTicks) < 0.001) {
    // not travelled
    return _pose;
  }

  double distance = (newWheelTicks / _ticksPerRound) * _wheelDiameter * M_PI * CORRECTION_FACTOR;
  Angle diffAngle = zAngle - _previousAngle;

  if (diffAngle.rad() != 0) {
    double radius = distance / diffAngle.rad();
    if (radius < MIN_RADIUS && radius > -MIN_RADIUS) {
      int sign = radius > 0 ? 1 : -1;
      diffAngle = Angle::rad(distance / (sign * MIN_RADIUS));
    }
  }

  _pose *= Geometry::calculateArcPose(distance, diffAngle);

  _previousTicks = ticks;
  _previousAngle = zAngle;

  return _pose;
}
