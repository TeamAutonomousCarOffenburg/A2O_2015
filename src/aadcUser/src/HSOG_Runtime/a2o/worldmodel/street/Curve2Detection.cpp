#include "Curve2Detection.h"
#include <utils/geometry/Geometry.h>


using namespace A2O;
using namespace Eigen;

#define MAX_ORIGIN_DISTANCE 0.8
#define MAX_RADIUS_DIFFERENCE 1.5

Curve2Detection::Curve2Detection(const Circle2D& circle,
			       const bool& leftCurve)
      : SegmentDetection(1), _circle(circle), _leftCurve(leftCurve)
{
}

Curve2Detection::~Curve2Detection()
{
}

const Circle2D& Curve2Detection::getCircle() const
{
  return _circle;
}

const bool& Curve2Detection::isLeftCurve() const
{
  return _leftCurve;
}

void Curve2Detection::add(const Circle2D& circle)
{
  _circle = Circle2D::avg(_circle, circle, _detectionsCount, 1.0);
  incrementDetectionsCount();
}

const double Curve2Detection::getOriginDistanceTo(const Circle2D& circle) const
{
  return Geometry::getDistance(_circle.origin(), circle.origin());
}

const double Curve2Detection::getAbsRadiusDistanceTo(const Circle2D& circle) const
{
  return std::fabs(_circle.radius() - circle.radius());
}
