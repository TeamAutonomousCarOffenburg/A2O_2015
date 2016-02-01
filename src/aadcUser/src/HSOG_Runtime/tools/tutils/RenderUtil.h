#pragma once

#include <QtCore/QtCore>
#include <QtGui/QtGui>

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


/**
 * Some rendering utils.
 *
 * \author Stefan Glaser
 */
class RenderUtil {
public:
  static void extractPath(const std::vector<Eigen::Vector2d>& points,
			  QPainterPath& path,
			  const bool& closePath = true);
  static void extractPath(const std::vector<QPointF>& points,
			  QPainterPath& path,
			  const bool& closePath = true);
  
  static void drawTriangle(QPainter& painter,
			   const QPointF& p1,
			   const QPointF& p2,
			   const QPointF& p3);
  static void fillTriangle(QPainter& painter,
			   const QPointF& p1,
			   const QPointF& p2,
			   const QPointF& p3,
			   const QBrush& fillBrush);
  static void fillAndDrawTriangle(QPainter& painter,
				  const QPointF& p1,
				  const QPointF& p2,
				  const QPointF& p3,
				  const QBrush& fillBrush);
  
  
  static void drawPoseArrow(QPainter& painter,
			    const QBrush& fillBrush);
  
  static void drawPolygon(QPainter& painter,
			  const std::vector<Eigen::Vector2d>& poly);
  static void fillPolygon(QPainter& painter,
			  const std::vector<Eigen::Vector2d>& poly,
			  const QBrush& fillBrush);
  static void fillAndDrawPolygon(QPainter& painter,
				 const std::vector<Eigen::Vector2d>& poly,
				 const QBrush& fillBrush);
  
  static const double drawLineSegment(QPainter& painter,
				      const double& length,
				      const double& exitAngle);
  static const double drawLineSegment(QPainter& painter,
				      const double& length,
				      const double& exitAngle,
				      QPen& helperPen);
  static void drawArc(QPainter& painter,
		      const QPointF& point,
		      const double& radius,
		      const double& angle);
  static void drawArc(QPainter& painter,
		      const QPointF& point,
		      const double& radius,
		      const double& angle,
		      QPen& helperPen);
  
private:
  static const QPointF ZERO;
};
