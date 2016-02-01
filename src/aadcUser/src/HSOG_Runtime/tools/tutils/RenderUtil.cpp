#include "RenderUtil.h"
#include <utils/geometry/Geometry.h>

#include <cmath>
#include <iostream>


using namespace A2O;
using namespace Eigen;

const QPointF ZERO = QPointF(0, 0);

void RenderUtil::extractPath(const std::vector<Vector2d>& points,
			     QPainterPath& path,
			     const bool& closePath)
{
  if (points.size() < 2) {
    return;
  }
  
  path.moveTo(QPointF(points[0](0), points[0](1)));
  for (Vector2d p : points) {
    path.lineTo(QPointF(p(0), p(1)));
  }
  if (closePath) {
    path.lineTo(QPointF(points[0](0), points[0](1)));
  }
}

void RenderUtil::extractPath(const std::vector<QPointF>& points,
			     QPainterPath& path,
			     const bool& closePath)
{
  if (points.size() < 2) {
    return;
  }
  
  path.moveTo(points[0]);
  for (QPointF p : points) {
    path.lineTo(p);
  }
  if (closePath) {
    path.lineTo(points[0]);
  }
}

void RenderUtil::drawTriangle(QPainter& painter,
			      const QPointF& p1,
			      const QPointF& p2,
			      const QPointF& p3)
{
  QPainterPath path;
  path.moveTo(p1);
  path.lineTo(p2);
  path.lineTo(p3);
  
  painter.drawPath(path);
}

void RenderUtil::fillTriangle(QPainter& painter,
			      const QPointF& p1,
			      const QPointF& p2,
			      const QPointF& p3,
			      const QBrush& fillBrush)
{
  QPainterPath path;
  path.moveTo(p1);
  path.lineTo(p2);
  path.lineTo(p3);
  
  painter.fillPath(path, fillBrush);
}

void RenderUtil::fillAndDrawTriangle(QPainter& painter,
				     const QPointF& p1,
				     const QPointF& p2,
				     const QPointF& p3,
				     const QBrush& fillBrush)
{
  QPainterPath path;
  path.moveTo(p1);
  path.lineTo(p2);
  path.lineTo(p3);
  
  painter.fillPath(path, fillBrush);
  painter.drawPath(path);
}

void RenderUtil::drawPoseArrow(QPainter& painter,
			       const QBrush& fillBrush)
{
  painter.drawLine(QPointF(-0.1, 0), QPointF(0.15, 0));
  painter.drawLine(QPointF(0, 0.1), QPointF(0, -0.1));
  
  QPainterPath path;
  path.moveTo(QPointF(0.1, 0.05));
  path.lineTo(QPointF(0.25, 0));
  path.lineTo(QPointF(0.1, -0.05));

  painter.fillPath(path, fillBrush);
}

void RenderUtil::drawPolygon(QPainter& painter,
			     const std::vector<Vector2d>& poly)
{
  if (poly.size() < 3) {
    return;
  }
  
  QPainterPath path;
  extractPath(poly, path);
  painter.drawPath(path);
}

void RenderUtil::fillPolygon(QPainter& painter,
			     const std::vector<Vector2d>& poly,
			     const QBrush& fillBrush)
{
  if (poly.size() < 3) {
    return;
  }
  
  QPainterPath path;
  extractPath(poly, path);
  painter.fillPath(path, fillBrush);
}

void RenderUtil::fillAndDrawPolygon(QPainter& painter,
				    const std::vector<Vector2d>& poly,
				    const QBrush& fillBrush)
{
  if (poly.size() < 3) {
    return;
  }
  
  QPainterPath path;
  extractPath(poly, path);
  painter.fillPath(path, fillBrush);
  painter.drawPath(path);
}

const double RenderUtil::drawLineSegment(QPainter& painter,
					 const double& length,
					 const double& exitAngle)
{
  double radius = 0;
  double angle = std::fmod(exitAngle, 2 * M_PI);
  int extension = int(angle * 16 * 180 / M_PI);
  if (std::abs(extension) == 0) {
    painter.drawLine(QPointF(0, 0), QPointF(length, 0));
  } else {
    radius = length / angle;
    painter.drawArc(QRectF(-radius, 0, 2 * radius, 2 * radius), radius < 0 ? 4320 : 1440, -extension);
  }
  
  return radius;
}

const double RenderUtil::drawLineSegment(QPainter& painter,
				 const double& length,
				 const double& exitAngle,
				 QPen& helperPen)
{
  double radius = drawLineSegment(painter, length, exitAngle);
  
  if (std::fabs(radius) > 0.0001) {
    double x = std::sin(exitAngle) * radius;
    double y = (1 - std::cos(exitAngle)) * radius;
    
    QPen oldPen = painter.pen();
    painter.setPen(helperPen);
    QPointF origin(0, radius);
    painter.drawLine(QPointF(0, 0), origin);
    painter.drawLine(QPointF(x, y), origin);
    painter.setPen(oldPen);
  }
  
  return radius;
}

void RenderUtil::drawArc(QPainter& painter,
			 const QPointF& point,
			 const double& radius,
			 const double& angle)
{
  double a = std::fmod(angle, 2 * M_PI);
  int extension = int(a * 16 * 180 / M_PI);
  if (std::abs(extension) == 0) {
    painter.drawLine(QPointF(0, 0), QPointF(std::sin(angle) * radius, 0));
  } else {
    painter.drawArc(QRectF(point.x() - radius, point.y(), 2 * radius, 2 * radius), radius < 0 ? 4320 : 1440, -extension);
  }
}

void RenderUtil::drawArc(QPainter& painter,
			 const QPointF& point,
			 const double& radius,
			 const double& angle,
			 QPen& helperPen)
{
  double a = std::fmod(angle, 2 * M_PI);
  int extension = int(a * 16 * 180 / M_PI);
  if (std::abs(extension) == 0) {
    painter.drawLine(QPointF(0, 0), QPointF(std::sin(angle) * radius, 0));
  } else {
    double x = std::sin(a) * radius;
    double y = (1 - std::cos(a)) * radius;
    QPointF origin(point.x(), point.y() + radius);
    QPen oldPen = painter.pen();
    painter.setPen(helperPen);
    painter.drawLine(point, origin);
    painter.drawLine(QPointF(point.x() + x, point.y() + y), origin);
    painter.setPen(oldPen);
    painter.drawArc(QRectF(point.x() - radius, point.y(), 2 * radius, 2 * radius), radius < 0 ? 4320 : 1440, -extension);
  }
}
