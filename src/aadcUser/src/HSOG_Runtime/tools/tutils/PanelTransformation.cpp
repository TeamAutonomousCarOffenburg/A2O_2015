#include "PanelTransformation.h"


#define MIN_SCALE 10.0


PanelTransformation::PanelTransformation()
      : _halfPanelWidth(9.5), _halfPanelHeight(9.5), _xShift(0), _yShift(0), _scale(10)
{
  computeTransformation();
}

PanelTransformation::~PanelTransformation()
{

}

const float& PanelTransformation::getHalfPanelWidth() const
{
  return _halfPanelWidth;
}

const float& PanelTransformation::getHalfPanelHeight() const
{
  return _halfPanelHeight;
}

const double& PanelTransformation::getXShift() const
{
  return _xShift;
}

const double& PanelTransformation::getYShift() const
{
  return _yShift;
}

const double& PanelTransformation::getScale() const
{
  return _scale;
}

const QTransform& PanelTransformation::getTransform() const
{
  return _trans;
}

void PanelTransformation::update(const int& panelWidth, const int& panelHeight)
{
  _halfPanelWidth = float(panelWidth - 1) / 2.0;
  _halfPanelHeight = float(panelHeight - 1)  / 2.0;
  
  computeTransformation();
}

void PanelTransformation::setShift(const double& xShift, const double& yShift)
{
  _xShift = xShift;
  _yShift = yShift;
  computeTransformation();
}

void PanelTransformation::setScale(const double& scale)
{
  if (scale > 0) {
    _scale = scale;
    computeTransformation();
  }
}

void PanelTransformation::zoomIn(const int& steps)
{
  zoom(true, steps);
}

void PanelTransformation::zoomOut(const int& steps)
{
  zoom(false, steps);
}

void PanelTransformation::zoom(const bool& in,
			       const int& steps)
{
  if (in) {
    _scale += steps;
  } else {
    _scale -= steps;
  }
  
  if (_scale < MIN_SCALE) {
    _scale = MIN_SCALE;
  }
  
  computeTransformation();
}

void PanelTransformation::shiftPixel(const int& vx,
				     const int& vy)
{
  shift(vy / _scale, vx / _scale);
}

void PanelTransformation::shiftPixel(const QPoint& vector)
{
  shiftPixel(vector.x(), vector.y());
}

void PanelTransformation::shift(const double& x,
				const double& y)
{
  _xShift += x;
  _yShift += y;
  computeTransformation();
}

void PanelTransformation::shift(const QPointF& vector)
{
  shift(vector.x(), vector.y());
}

const QTransform PanelTransformation::computeTransformationFor(const A2O::Pose2D& pose) const
{
  double sinAngle = pose.getAngle().sin();
  double cosAngle = pose.getAngle().cos();
  QTransform poseTrans(cosAngle, sinAngle, -sinAngle, cosAngle, pose.x(), pose.y());
  return poseTrans * _trans;
}

void PanelTransformation::setTransform(const A2O::Pose2D& pose,
				       QPainter& painter) const
{
  painter.setTransform(computeTransformationFor(pose));
}

void PanelTransformation::setTransform(QPainter& painter) const
{
  painter.setTransform(_trans);
}

void PanelTransformation::computeTransformation()
{
  double tx = _scale * _yShift + _halfPanelWidth;
  double ty = _scale * _xShift + _halfPanelHeight;
  _trans = QTransform(0, -_scale, -_scale, 0, tx, ty);
}
