#pragma once

#include <QtCore/QtCore>
#include <QtGui/QtGui>

#include "worldmodel/IWorldModel.h"
#include "carmodel/ICarModel.h"
#include "worldmodel/street/Segment.h"
#include "worldmodel/street/SegmentLink.h"
#include "worldmodel/lanedetection/LineDetection.h"
#include "worldmodel/lanedetection/CurveDetection.h"
#include "utils/geometry/LineSegment.h"
#include "utils/geometry/Polygon.h"
#include <tutils/PanelTransformation.h>
#include "renderer/StreetRenderer.h"
#include "renderer/ThisCarRenderer.h"


/**
 * The WorldViewer is the central qt-widget for visualization of our internal state within ADTF.
 *
 * \author Stefan Glaser
 */
class WorldViewer : public QWidget
{
  Q_OBJECT
public:
  WorldViewer(QWidget *parent = 0);
  virtual ~WorldViewer(){};
  
  void setCarModel(A2O::ICarModel::Ptr car);
  void setWorldModel(A2O::IWorldModel::Ptr world);
  
  virtual void paintEvent(QPaintEvent* evt);
  virtual void wheelEvent(QWheelEvent* evt);
  virtual void mousePressEvent(QMouseEvent* evt);
  virtual void keyPressEvent(QKeyEvent *evt);
  virtual void mouseReleaseEvent(QMouseEvent* evt);
  virtual void mouseMoveEvent(QMouseEvent* evt);

private:
  void drawSegment(QPainter& painter, A2O::Segment::Ptr segment);
  void drawSegmentLink(QPainter& painter, A2O::SegmentLink::Ptr link);
  void drawPose(QPainter& painter, A2O::Pose2D pose, const QPen& pen, bool withText = false);
  void drawLineSegment(QPainter& painter, const A2O::LineSegment& segment);
  void drawCar(QPainter& painter);
  void drawGrid(QPainter& painter);
  void drawStatusLine(QPainter& painter);
  void drawDirectionsClock(QPainter& painter);
  void drawSpeedometer(QPainter& painter);
  void drawLineDetector(QPainter& painter);
  void drawLineDetection(QPainter& painter, const A2O::LineDetection& line);
  void drawCurveDetection(QPainter& painter, const A2O::CurveDetection& line);
  void drawRoadSigns(QPainter& painter);
  void drawObjects(QPainter& painter);
  
  void drawPolygon(QPainter& painter,
		   A2O::Polygon::Ptr poly,
		   const QPen& borderPen);
  void drawAndFillPolygon(QPainter& painter,
		   A2O::Polygon::Ptr poly,
		   const QPen& borderPen,
		   const QPen& fillPen);
  
  void drawTriangle(QPainter& painter, const QPoint& p1, const QPoint& p2, const QPoint& p3, const QPen& pen);
  
  void extractPath(const std::vector<Eigen::Vector2d>& points,
		   QPainterPath& path,
		   const bool& closePath = true);
  
  void drawTrail(QPainter& painter);
  
  QPoint worldToPanel(const double& x, const double& y);
  QPoint worldToPanel(const Eigen::Vector2d& pos);
  int worldToPanel(const double& length);
  int worldToPanel(const A2O::Angle& angle);
  
  QPointF panelToWorld(const int& x, const int& y);
  QPointF panelToWorld(const QPoint& pos);
  
  QString getQString(const A2O::Pose2D& pose);
  QString getQString(const double& value, const int& precision = 3);
  QString getQString(const int& value);

  void showLegend();
  QDialog *_legend;

  
  QPen _gridPen;
  QPen _carPen;
  QPen _carHelperPen;
  QPen _borderPen;
  QPen _centerPen;
  QPen _linkPen;
  QPen _intendedLinkPen;
  QPen _helperLinePen;
  QPen _supportPosePen;
  QPen _wayPointPen;
  QPen _trailPen; 
  
  QPen _clockBackgroundPen;
  QPen _clockBorderPen;
  QPen _gyroPen;

  // lines
  QPen _leftPen;
  QPen _endLeftPen;
  QPen _startLeftPen;
  QPen _upperLeftPen;
  QPen _upperLeftStartPen;
  
  
  QPen _rightPen;
  QPen _endRightPen;
  QPen _rightStartPen;
  QPen _stopRightPen;
  QPen _stopRightMiddlePen;
  QPen _stopPen;
  QPen _stopMiddlePen;
 
  QPen _upperRightPen;
  QPen _upperRightStartPen;
  QPen _leftCurvePen;
  QPen _rightCurvePen;
  
  double _scale;
  double _xShift;
  double _yShift;

  bool _drawSignImages;

  QPointF _dragPos;
  QPoint _rightClickPos;
  bool _dragging;
  bool _settingWayPoint;
  QTimer* _timer;
  
  int _halfWidth;
  int _halfHeight;

  std::vector<std::pair<QPen, std::string>> _pens;
  void registerPen(QPen& pen, const QColor color, std::string name);

  A2O::IWorldModel::Ptr _worldModel = nullptr;
  A2O::ICarModel::Ptr _carModel = nullptr;
  
  PanelTransformation _panelTransform;
  
  StreetRenderer _streetRenderer;
  ThisCarRenderer _thisCarRenderer;
};
