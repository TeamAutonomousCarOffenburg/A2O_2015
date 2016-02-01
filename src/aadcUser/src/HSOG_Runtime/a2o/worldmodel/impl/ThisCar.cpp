#include "ThisCar.h"


using namespace A2O;
using namespace Eigen;

ThisCar::ThisCar(const std::string& name,
		 const Pose2D& pose,
		 Polygon::Ptr bounds)
      : Car(name, pose),
        _path(boost::make_shared<Path2D>()),
        _pathPos(0)
{
  if (bounds) {
    _bounds = bounds;
  } else {
    _bounds = boost::make_shared<Polygon>(Vector2d(0.473, 0.15), Vector2d(0.473, -0.15), Vector2d(-0.113, -0.15), Vector2d(-0.113, 0.15));
  }
}

ThisCar::~ThisCar()
{
}

void ThisCar::update(const Pose2D& pose)
{
  // Call super class update
  Car::update(pose);
  
  // Update path information
  _pathPos = _path->getPositionOnPath(_pose.getPosition());
}

void ThisCar::update(const Pose2D& pose,
		     Path2D::Ptr path)
{
  // Call super class update
  Car::update(pose);
  
  _path = path;
  
  // Update path information
  _pathPos = _path->getPositionOnPath(_pose.getPosition());
}

void ThisCar::setPose(const Pose2D& pose)
{
  _pose = pose;
  
  // Update path information
  _pathPos = _path->getPositionOnPath(_pose.getPosition());
}

Path2D::ConstPtr ThisCar::getPath() const
{
  return boost::dynamic_pointer_cast<const Path2D>(_path);
}

void ThisCar::setPath(Path2D::Ptr path)
{
  _path = path;
  
  // Update path information
  _pathPos = _path->getPositionOnPath(_pose.getPosition());
}

const double& ThisCar::getPathPosition() const
{
  return _pathPos;
}
