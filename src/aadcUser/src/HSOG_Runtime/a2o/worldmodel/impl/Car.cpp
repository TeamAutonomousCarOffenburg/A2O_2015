#include "Car.h"


using namespace A2O;
using namespace Eigen;

Car::Car(const std::string& name,
	 const Pose2D& pose,
	 Polygon::Ptr bounds)
      : MovableObject(name, pose)
{
  if (bounds) {
    _bounds = bounds;
  } else {
    _bounds = boost::make_shared<Polygon>(Vector2d(0.3, 0.15), Vector2d(0.3, -0.15), Vector2d(-0.3, -0.15), Vector2d(-0.3, 0.15));
  }
}

Car::~Car()
{
}
