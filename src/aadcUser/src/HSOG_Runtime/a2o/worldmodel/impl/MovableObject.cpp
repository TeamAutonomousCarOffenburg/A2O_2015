#include "MovableObject.h"


using namespace A2O;
using namespace Eigen;

MovableObject::MovableObject(const std::string& name,
			     const Pose2D& pose)
      : VisibleObject(name, pose), _speed(0, 0)
{

}

MovableObject::~MovableObject()
{
}

void MovableObject::update(const Pose2D& pose)
{
  Pose2D previousPose = _pose;
  
  // Call update of visible object
  VisibleObject::update(pose);
  
  // Calculate object speed
  _speed = _pose.getPosition() - previousPose.getPosition();
}

const Vector2d& MovableObject::getSpeed() const
{
  return _speed;
}
