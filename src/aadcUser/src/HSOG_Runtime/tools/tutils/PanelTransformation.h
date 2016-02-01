#pragma once

#include <QtCore/QtCore>
#include <QtGui/QtGui>
#include <utils/geometry/Pose2D.h>

#include <boost/smart_ptr.hpp>


/**
 * The PanelTransformation class provides transformation routines for panel-to-world coordinates and vice versa.
 * Furthermore, it features an simple interface for zooming, shifting, etc.
 *
 * \author Stefan Glaser
 */
class PanelTransformation {
public:
  typedef boost::shared_ptr<PanelTransformation> Ptr;
  typedef boost::shared_ptr<const PanelTransformation> ConstPtr;
  
  PanelTransformation();
  ~PanelTransformation();
  
  const float& getHalfPanelWidth() const;
  const float& getHalfPanelHeight() const;
  
  const double& getXShift() const;
  const double& getYShift() const;
  const double& getScale() const;
  
  const QTransform& getTransform() const;
  
  void update(const int& panelWidth,
	      const int& panelHeight);
  
  void setShift(const double& xShift,
		const double& yShift);
  
  void setScale(const double& scale);
  
  void zoom(const bool& in, const int& steps = 10);
  void zoomIn(const int& steps = 10);
  void zoomOut(const int& steps = 10);
  
  void shiftPixel(const int& vx,
		  const int& vy);
  void shiftPixel(const QPoint& vector);
  void shift(const double& x,
	     const double& y);
  void shift(const QPointF& vector);
  
  const QTransform computeTransformationFor(const A2O::Pose2D& pose) const;
  
  void setTransform(QPainter& painter) const;
  void setTransform(const A2O::Pose2D& pose,
		    QPainter& painter) const;
  
private:
  void computeTransformation();
  
  float _halfPanelWidth;
  float _halfPanelHeight;
  double _xShift;
  double _yShift;
  double _scale;
  
  QTransform _trans;
};
