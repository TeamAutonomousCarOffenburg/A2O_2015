#include "WorldViewer.h"

#include "renderer/StreetRenderer.h"
#include "renderer/ThisCarRenderer.h"
#include "tutils/RenderUtil.h"

#include <utils/geometry/Geometry.h>
#include <utils/geometry/Pose2D.h>
#include <utils/geometry/LineSegment.h>
#include <worldmodel/impl/WayPointExtractor.h>
#include <worldmodel/IDriveInstructionConstants.h>
#include <AADCCar.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace A2O;
using namespace Eigen;
using namespace std;

WorldViewer::WorldViewer(QWidget* parent)
      : QWidget(parent), _scale(100), _xShift(2), _yShift(0)
{
  _gridPen = QPen(Qt::DotLine);
  _gridPen.setColor(QColor("#bbbbbb"));
  
  registerPen(_carPen, Qt::red, "Car");
  _carPen.setWidth(2);
  
  _carHelperPen = QPen(Qt::DashLine);
  _carHelperPen.setColor(Qt::red);
 
  registerPen(_trailPen, Qt::green, "Trail"); 
  _trailPen.setWidth(3);

  _borderPen = QPen(QColor("#222222"));
  _centerPen = QPen(QColor("#aaaaaa"));
  _linkPen = QPen(Qt::blue);
  registerPen(_intendedLinkPen, Qt::darkGreen, "Intended Link");
  _helperLinePen = QPen(Qt::DashDotLine);
  _helperLinePen.setColor(QColor("#999999"));
  registerPen(_supportPosePen, Qt::lightGray, "Support Pose");
  registerPen(_wayPointPen, QColor("#00bb00"), "WayPoint");

  // lines
  registerPen(_leftPen, Qt::magenta, "Left Line");
  registerPen(_endLeftPen, Qt::darkMagenta, "End Left Line");
  registerPen(_startLeftPen, Qt::blue, "Start Left Line");
  registerPen(_upperLeftPen, Qt::darkRed, "Upper Left Line");
  registerPen(_upperLeftStartPen, Qt::green, "Upper Left Start Line");
  
  registerPen(_rightPen, Qt::blue, "Right Line");
  registerPen(_endRightPen, Qt::cyan, "End Right Line");
  registerPen(_rightStartPen, Qt::darkCyan, "Right Start Line");
  registerPen(_stopRightPen, Qt::green, "Right Stop Line");
  registerPen(_stopRightMiddlePen, Qt::darkGreen, "Right Stop Middle Line");
  registerPen(_stopPen, Qt::red, "Stop Line"); 
  registerPen(_stopMiddlePen, Qt::magenta, "Stop Middle Line");

  registerPen(_upperRightPen, Qt::darkBlue, "Right Upper Line");
  registerPen(_upperRightStartPen, Qt::darkCyan, "Right Upper Start Line");

  registerPen(_leftCurvePen, Qt::green, "Left Curve Line");
  registerPen(_rightCurvePen, Qt::darkGreen, "Right Curve Line");

  _clockBackgroundPen = QPen(Qt::SolidPattern);
  _clockBackgroundPen.setColor(Qt::white);
  _gyroPen = QPen(Qt::darkGreen);
  _gyroPen.setWidth(2);
  _clockBorderPen = QPen(Qt::darkGray);

  _drawSignImages = false;
  const int TimerFreq = 1000/33;

  _timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), this, SLOT(update()));
  _timer->start(TimerFreq);
  _panelTransform.setScale(_scale);
  _panelTransform.setShift(_xShift, _yShift);
}

void WorldViewer::setCarModel(ICarModel::Ptr car)
{
  _carModel = car;
}

void WorldViewer::setWorldModel(IWorldModel::Ptr world)
{
  _worldModel = world;
}

void WorldViewer::wheelEvent(QWheelEvent* evt)
{
  int steps = evt->delta() / 20;
  _scale += steps;
  if (_scale < 10) {
    _scale = 10;
  }
  
  _panelTransform.setScale(_scale);
  
  evt->accept();
  update();
}

void WorldViewer::showLegend()
{
	_legend = new QDialog();
	QTableWidget *table = new QTableWidget(_pens.size(),2, _legend);
	table->setHorizontalHeaderItem(0, new QTableWidgetItem("Name"));
	table->setHorizontalHeaderItem(1, new QTableWidgetItem("Color"));
	for(unsigned int i=0; i < _pens.size(); i++)
	{
		table->setItem(i, 0, new QTableWidgetItem);
		table->setItem(i, 1, new QTableWidgetItem);
		table->item(i, 0)->setText(QString::fromStdString(_pens[i].second));
		table->item(i, 1)->setBackground(_pens[i].first.color());
	}
	QHBoxLayout *layout = new QHBoxLayout();
	layout->addWidget(table);
	_legend->setLayout(layout);
	_legend->show();
}

void WorldViewer::registerPen(QPen& pen, const QColor color, std::string name)
{
	pen.setColor(color);
	_pens.push_back(make_pair(pen,name));
}

void WorldViewer::keyPressEvent(QKeyEvent *evt)
{
	cout << "got keyevent qt" << endl;
	if(evt->key() == Qt::Key_L)
	{
		showLegend();
	}
}

void WorldViewer::mousePressEvent(QMouseEvent* evt)
{
	if(evt->button() == Qt::LeftButton)
	{
  		_dragging = true;
  		_dragPos = QPointF(evt->y() / _scale, evt->x() / _scale);
	}
	else if(evt->button() == Qt::RightButton)
	{
		_settingWayPoint = true;
		_rightClickPos = evt->pos();
	}
}

void WorldViewer::mouseReleaseEvent(QMouseEvent* evt)
{
	if(evt->button() == Qt::LeftButton)
	{
		_dragging = false;
	}
	else if(evt->button() == Qt::RightButton)
	{
		_settingWayPoint = false;
		QPointF goal = panelToWorld(_rightClickPos);	

		QPointF release = panelToWorld(evt->pos());

		QPointF diff = release - goal;

		Angle angle = Angle::to(diff.x(), diff.y());

		Path2D::Ptr path(boost::make_shared<Path2D>());
		path->addWayPoint(_worldModel->getThisCar()->getPose());
		path->addWayPoint(Pose2D(goal.x(),goal.y(), angle));
		_worldModel->getThisCar()->setPath(path);
	}
}

void WorldViewer::mouseMoveEvent(QMouseEvent* evt)
{
  if (_dragging) {
    QPointF newPos(evt->y() / _scale, evt->x() / _scale);
    QPointF dragDiff = newPos - _dragPos;
    
    _xShift += dragDiff.x();
    _yShift += dragDiff.y();
    
    _panelTransform.setShift(_xShift, _yShift);
    
    _dragPos = newPos;
    update();
  }
}

void WorldViewer::paintEvent(QPaintEvent* evt)
{
  QPainter painter(this);

  _halfWidth = width() / 2;
  _halfHeight = height() / 2;
  
  _panelTransform.update(width(), height());

  painter.translate(_halfWidth, _halfHeight);

  drawGrid(painter);
 
  /*
  Segment::Ptr seg = Segment::createInitialSegment(Pose2D(0, 0, Angle::deg(0)), 1);
 
  vector<RoadSign::Ptr> signs;
  signs.push_back(boost::make_shared<RoadSign>(Stop, Pose2D(1,1,0)));
  seg->attachRoadSigns(signs);

  Segment::Ptr newSeg = Segment::appendSegment(seg->getStraightOption(), 2);
  newSeg = Segment::appendXCrossing(newSeg->getStraightOption());
  newSeg = Segment::appendCurve2MLeft(newSeg->getStraightOption()->getSegmentAfter()->getStraightOption());
  newSeg = Segment::appendXCrossing(newSeg->getStraightOption());
  newSeg = Segment::appendSegment(newSeg->getRightOption()->getSegmentAfter()->getStraightOption(), 2, Angle::deg(-40));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), 1.5, Angle::deg(-110));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), 0.8, Angle::deg(60));
  newSeg = Segment::appendTCrossingLR(newSeg->getStraightOption());
  drawSegment(painter, seg);
  
  vector<string> driveInstructions;
  driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_STRAIGHT);
  driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_RIGHT);
  driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_RIGHT);
  DriveInstructionManager::Ptr mngr(boost::make_shared<DriveInstructionManager>(driveInstructions));

  vector<Pose2D> staticWayPoints;
  WayPointExtractor::extractWayPoints(seg, mngr, staticWayPoints);
  for (size_t i = 0; i < staticWayPoints.size(); i++) {
	  drawPose(painter, staticWayPoints[i], _wayPointPen);
  }
  */
  
//   QTransform qTrans(0, -_scale, -_scale, 0, _scale*_yShift + _halfWidth, _scale*_xShift + _halfHeight);
//   
//   Pose2D pose(1, 0, Angle::deg(-10));
//   double sinAngle = -pose.getAngle().sin();
//   double cosAngle = pose.getAngle().cos();
//   QTransform poseTrans(cosAngle, -sinAngle, sinAngle, cosAngle, pose.x(), pose.y());
//   QTransform qTarget = poseTrans * qTrans;
//   
//   painter.setTransform(qTarget);
//   
//   _carPen.setWidthF(0.01);
// //   _carHelperPen.setWidthF(0.01);
//   painter.setPen(_carPen);
// //   painter.drawLine(0, 0, 3, 0);
//   painter.drawLine(0, 0, 2, 2);
//   painter.drawLine(0, 0, 2, -2);
//   
//   RenderUtil::drawLineSegment(painter, 5, 5 * M_PI / 4, _carHelperPen);
//   painter.setPen(_carPen);
//   RenderUtil::drawLineSegment(painter, 5, -5 * M_PI / 4);
// //   painter.setPen(_carPen);
// //   RenderUtil::drawLineSegment(painter, 5, -0.000001);
//   painter.setPen(_carPen);
//   RenderUtil::drawPoseArrow(painter, _carPen.brush());

/*
  PanelTransformation pt;
  pt.update(width(), height());
  pt.setScale(_scale);
  pt.setShift(_xShift, _yShift);
  StreetRenderer sr;
  sr.render(_worldModel, _carModel, painter, pt);
  
  ThisCarRenderer tcr;
  tcr.render(_worldModel, _carModel, painter, pt);

*/
  if (!_worldModel || !_carModel) {
    return;
  }

  if (_worldModel->isInitialized()) {
    // Draw segment tree
    Segment::Ptr curSegment = _worldModel->getCurrentWorldState()->getCurrentSegment();
    if (curSegment) {
      drawSegment(painter, curSegment);
    }

    // Draw way points on segments
    const std::vector<Pose2D> wayPoints = _worldModel->getThisCar()->getPath()->getWayPoints();
    for (size_t i = 0; i < wayPoints.size(); i++) {
      drawPose(painter, wayPoints[i], _wayPointPen);
    }

    // Draw double-arc connector segments
    const std::vector<LineSegment>& segments = _worldModel->getThisCar()->getPath()->getLineSegments();
    for (LineSegment seg : segments) {
      drawLineSegment(painter, seg);
    }
  }
//   _streetRenderer.render(_worldModel, _carModel, painter, _panelTransform);

  drawLineDetector(painter);

  // Draw odometry poses
  drawCar(painter);
//   _thisCarRenderer.render(_worldModel, _carModel, painter, _panelTransform);

  // Draw virtual trail
  drawTrail(painter);
  
  drawRoadSigns(painter);
  // Draw status line
  drawStatusLine(painter);

  QPen pen = QPen(Qt::gray);
  pen.setWidth(10);
  painter.setPen(pen);
  painter.drawPoint(worldToPanel(_worldModel->getVirtualWayPoint()));

//  drawObjects(painter);

  // Paint Gyro clock
  painter.resetTransform();
  drawDirectionsClock(painter);
  
  painter.resetTransform();
  drawSpeedometer(painter);
}

void WorldViewer::drawGrid(QPainter& painter)
{
  QPoint centerPos = QPoint(int(_xShift - (_halfHeight / _scale)), int(_yShift - (_halfWidth / _scale)));
  QPoint runningPos = worldToPanel(centerPos.x(), centerPos.y());

  do {
    painter.setPen(_gridPen);
    painter.drawLine(runningPos.x(), -_halfHeight, runningPos.x(), _halfHeight);
    painter.drawLine(-_halfWidth, runningPos.y(), _halfWidth, runningPos.y());
    
    // Draw labels
    painter.setPen(Qt::darkGray);
    painter.drawText(2 - _halfWidth, runningPos.y() - 2, getQString(centerPos.x()));
    painter.drawText(runningPos.x() + 2, -_halfHeight + 15, getQString(centerPos.y()));

    centerPos.rx()++;
    centerPos.ry()++;
    runningPos = worldToPanel(centerPos.x(), centerPos.y());
  } while ((runningPos.x() >= -_halfWidth && runningPos.x() <= _halfWidth) || (runningPos.y() >= -_halfHeight && runningPos.y() <= _halfHeight));

  // Draw coordinate axis
  runningPos = worldToPanel(0, 0);

  painter.setPen(Qt::darkGray);
  if (runningPos.x() >= -_halfWidth && runningPos.x() < _halfWidth) {
    painter.drawLine(runningPos.x(), -_halfHeight, runningPos.x(), _halfHeight);
  }
  if (runningPos.y() >= -_halfHeight && runningPos.y() < _halfHeight) {
    painter.drawLine(-_halfWidth, runningPos.y(), _halfWidth, runningPos.y());
  }
}

void WorldViewer::drawStatusLine(QPainter& painter)
{
  Pose2D pose1 = _worldModel->getThisCar()->getPose();

  stringstream poseLbl;
  poseLbl << "Car Pose:  (" << pose1.x() << " | " << pose1.y() << " | " << pose1.getAngle().deg() << ")";

  painter.setPen(Qt::black);
  painter.drawText(10 - _halfWidth, _halfHeight - 20, QString(poseLbl.str().c_str()));
  
  Eigen::Vector3d normalFloor = _worldModel->getFloorNormal();
  
  double normalLength = sqrt(normalFloor(0) * normalFloor(0) + normalFloor(1) * normalFloor(1) + normalFloor(2) * normalFloor(2));
//   cout << "x: " << normalFloor(0) << " y: " << normalFloor(1) << " z: " << normalFloor(2) << endl;
  stringstream floorNormalLbl;
  floorNormalLbl << "Floor Angle: zx: " << std::setprecision (2) << Angle::toDegrees( atan(normalFloor(0) / normalFloor(2))) 
  << " deg\tzy: " << std::setprecision (2) << Angle::toDegrees(atan( normalFloor(1) / normalFloor(2))) 
  << " deg\tcam z: " << std::setprecision (3) << normalLength << " m";
  painter.drawText(10 - _halfWidth, _halfHeight - 5, QString(floorNormalLbl.str().c_str()));
}

void WorldViewer::drawCar(QPainter& painter)
{
  Pose2D pose = _worldModel->getThisCar()->getPose();
  QPoint carPoint = worldToPanel(pose.getPosition());

  // Draw current steering
  painter.setPen(_carHelperPen);
  Angle steeringAngle = Angle::deg(_carModel->getServoDrive(AADC_Car::STEERING_SERVO)->getPosition());
  if (steeringAngle.rad() == 0) {
    QPoint targetPoint = worldToPanel(pose * Vector2d(1, 0));
    painter.drawLine(carPoint, targetPoint);
  } else {
    double radius = _carModel->getDriveCalculator()->calculateCurveRadius(steeringAngle);
    int iRadius = worldToPanel(radius);
    QPoint origin = worldToPanel(pose * Vector2d(0, radius));
    int startAngle;
    if (radius < 0) {
      startAngle = worldToPanel(pose.getAngle().opposite());
    } else {
      startAngle = worldToPanel(pose.getAngle());
    }
    double endAngle = Angle::toDegrees(1 / radius) * 16;
    if (endAngle > -1 && endAngle < 1) {
      if (endAngle < 0) {
	endAngle -= 1.0001;
      } else {
	endAngle += 1.0001;
      }
    }

    painter.drawArc(origin.x() - iRadius,
		    origin.y() - iRadius,
		    2 * iRadius,
		    2 * iRadius,
		    startAngle,
		    int(endAngle));
  }

  // Draw car pose (or direction indicators) on grid
  painter.setPen(_carPen);
  if (carPoint.y() > _halfHeight) {
    drawTriangle(painter,
		 QPoint(1 - _halfWidth, _halfHeight - 10),
		 QPoint(5 - _halfWidth, _halfHeight - 3),
		 QPoint(9 - _halfWidth, _halfHeight - 10),
		 _carPen);
  } else if (carPoint.y() < -_halfHeight) {
    drawTriangle(painter,
		 QPoint(1 - _halfWidth, 15 - _halfHeight),
		 QPoint(5 - _halfWidth, 8 - _halfHeight),
		 QPoint(9 - _halfWidth, 15 - _halfHeight),
		 _carPen);
  } else {
    painter.drawLine(-_halfWidth, carPoint.y(), 15 - _halfWidth, carPoint.y());
    painter.drawText(24 - _halfWidth, carPoint.y() + 4, getQString(pose.x()));
  }

  if (carPoint.x() > _halfWidth) {
    drawTriangle(painter,
		 QPoint(_halfWidth - 10, 1 - _halfHeight),
		 QPoint(_halfWidth - 3, 5 - _halfHeight),
		 QPoint(_halfWidth - 10, 9 - _halfHeight),
		 _carPen);
  } else if (carPoint.x() < -_halfWidth) {
    drawTriangle(painter,
		 QPoint(15 - _halfWidth, 1 - _halfHeight),
		 QPoint(8 - _halfWidth, 5 - _halfHeight),
		 QPoint(15 - _halfWidth, 9 - _halfHeight),
		 _carPen);
  } else {
    painter.drawLine(carPoint.x(), -_halfHeight, carPoint.x(), 20 - _halfHeight);
    painter.drawText(carPoint.x() + 6, 20 - _halfHeight, getQString(pose.y()));
  }

  // Draw car pose
  drawPose(painter, pose, _carPen);

  // Draw car box
  QPoint frontLeft = worldToPanel(pose * Vector2d(0.473, 0.15));
  QPoint frontRight = worldToPanel(pose * Vector2d(0.473, -0.15));
  QPoint backLeft = worldToPanel(pose * Vector2d(-0.113, 0.15));
  QPoint backRight = worldToPanel(pose * Vector2d(-0.113, -0.15));

  painter.drawLine(frontLeft, frontRight);
  painter.drawLine(frontRight, backRight);
  painter.drawLine(backRight, backLeft);
  painter.drawLine(backLeft, frontLeft);
}

void WorldViewer::drawDirectionsClock(QPainter& painter)
{
  painter.translate(100, 100);

  Angle carAngle = _worldModel->getThisCar()->getPose().getAngle();
  Angle gyroAngle = _carModel->getGyro(AADC_Car::CAR_GYRO)->getHorizontalAngle();

  // Negate angles since we are in a left hand system now
  carAngle = carAngle.negate();
  gyroAngle = gyroAngle.negate();

  QPainterPath circlePath;
  circlePath.moveTo(60, 0);
  circlePath.arcTo(-60, -60, 120, 120, 0, 600);

  // Fill a circle with border
  painter.fillPath(circlePath, _clockBackgroundPen.brush());
  painter.setPen(_clockBorderPen);
  painter.drawPath(circlePath);
  
  painter.drawText(-3, -63, "0");
  painter.drawText(65, 6, "-90");
  painter.drawText(-22, 74, "+/-180");
  painter.drawText(-80, 6, "90");

  // Draw helper lines
  painter.setPen(_gridPen);
  double x, y;
  for (int i = 0; i < 170; i += 45) {
    x = std::cos(i * M_PI / 180) * 60;
    y = std::sin(i * M_PI / 180) * 60;
    painter.drawLine(x, y, -x, -y);
  }

  // Draw direction indicator
  painter.setPen(_clockBorderPen);
  painter.drawText(-14, -45, "+    -");

  // Draw car angle
  painter.setPen(_carPen);
  x = std::cos(carAngle.rad() - M_PI / 2) * 60;
  y = std::sin(carAngle.rad() - M_PI / 2) * 60;
  painter.drawLine(0, 0, x, y);

  // Draw gyro angle
  painter.setPen(_gyroPen);
  x = std::cos(gyroAngle.rad() - M_PI / 2) * 60;
  y = std::sin(gyroAngle.rad() - M_PI / 2) * 60;
  painter.drawLine(0, 0, x, y);

  // Draw label backgrounds
  painter.setPen(_clockBorderPen);
  painter.fillRect(-18, -20, 36, -15, _clockBackgroundPen.brush());
  painter.fillRect(-18, 20, 36, 15, _clockBackgroundPen.brush());

  // Draw car angle label
  painter.setPen(_carPen);
  painter.drawText(-13, -22, getQString(-carAngle.deg(), 0));

  // Draw gyro angle label
  painter.setPen(_gyroPen);
  painter.drawText(-13, 33, getQString(-gyroAngle.deg(), 0));

  // Draw center point
  painter.setPen(_clockBorderPen);
  painter.drawPoint(0, 0);
}

void WorldViewer::drawSpeedometer(QPainter& painter)
{
  painter.translate(220, 100);
  double speed = _carModel->getMotor(AADC_Car::MAIN_MOTOR)->getSpeed() / 100.0;

  // Draw background
  painter.setPen(_clockBorderPen);
  painter.fillRect(-10, -60, 20, 120, _clockBackgroundPen.brush());

  // Draw speed indicator
  QPen speedPen = speed < 0 ? QPen(Qt::red) : _wayPointPen;
  painter.setPen(speedPen);
  int speedHeight = int(-60 * speed);
  painter.fillRect(-10, 0, 20, speedHeight, speedPen.brush());
  painter.drawLine(-10, speedHeight, 15, speedHeight);
  painter.drawText(20, speedHeight + 5, getQString(speed, 2));

  // Draw border
  painter.setPen(_clockBorderPen);
  painter.drawRect(-10, -60, 20, 120);
  painter.drawLine(-15, 0, 10, 0);
  painter.drawLine(-15, -60, -10, -60);
  painter.drawLine(-15, 60, -10, 60);
  
//   painter.drawText(-55, -50, "100%");
//   painter.drawText(-38, 6, "0%");
//   painter.drawText(-60, 60, "-100%");
  painter.drawText(-25, -50, "1");
  painter.drawText(-25, 6, "0");
  painter.drawText(-30, 60, "-1");
}

void WorldViewer::drawLineDetector(QPainter& painter)
{
  LineDetector::Ptr detector = _worldModel->getCurrentWorldState()->getCurrentLineDetector();
  if(!detector)
    return;
  
  painter.setPen(_leftPen);
  drawLineDetection(painter, detector->leftLine);
 
  painter.setPen(_endLeftPen);
  drawLineDetection(painter, detector->endLeft);
  
  painter.setPen(_upperLeftPen);
  drawLineDetection(painter, detector->upperLeft);

  painter.setPen(_startLeftPen);
  drawLineDetection(painter, detector->startLeft);

  painter.setPen(_upperLeftStartPen);
  drawLineDetection(painter, detector->upperLeftStart);
  
  painter.setPen(_rightPen);
  drawLineDetection(painter, detector->rightLine);
 
  painter.setPen(_endRightPen);
  drawLineDetection(painter, detector->endRight);
  
  painter.setPen(_rightStartPen);
  drawLineDetection(painter, detector->startRight);

  painter.setPen(_stopRightPen);
  drawLineDetection(painter, detector->stopRight);
 
  painter.setPen(_stopRightMiddlePen);
  drawLineDetection(painter, detector->stopRightMiddle);
  
  painter.setPen(_upperRightPen);
  drawLineDetection(painter, detector->upperRight);
 
  painter.setPen(_upperRightStartPen);
  drawLineDetection(painter, detector->upperRightStart);
    
  painter.setPen(_stopPen);
  drawLineDetection(painter, detector->stopLine);
 
  painter.setPen(_stopMiddlePen);
  drawLineDetection(painter, detector->stopLineMiddle);
 
  painter.setPen(_leftCurvePen);
  drawCurveDetection(painter, detector->leftCurve);
 
  painter.setPen(_rightCurvePen);
  drawCurveDetection(painter, detector->rightCurve);
   
}

void WorldViewer::drawCurveDetection(QPainter& painter, const CurveDetection& curve)
{
   if(!curve.valid)
	   return;

   QPointF start(worldToPanel(curve.startK(0), curve.startK(1)));
   QPointF middle(worldToPanel(curve.middleK(0), curve.middleK(1)));
   QPointF end(worldToPanel(curve.endK(0), curve.endK(1)));
   painter.drawLine(start, middle);
   painter.drawLine(middle, end);
}

void WorldViewer::drawLineDetection(QPainter& painter, const LineDetection& line)
{
   if(!line.valid)
     return;
   
   QPointF start(worldToPanel(line.startK(0), line.startK(1)));
   QPointF end(worldToPanel(line.endK(0), line.endK(1)));
   painter.drawLine(start, end);
}

void WorldViewer::drawPose(QPainter& painter,
			   Pose2D pose,
			   const QPen& pen,
			   bool withText)
{
  QPoint front = worldToPanel(pose * Vector2d(0.25, 0));
  QPoint back = worldToPanel(pose * Vector2d(-0.1, 0));
  QPoint left = worldToPanel(pose * Vector2d(0, 0.1));
  QPoint right = worldToPanel(pose * Vector2d(0, -0.1));
  QPoint arrLeft = worldToPanel(pose * Vector2d(0.1, 0.05));
  QPoint arrRight = worldToPanel(pose * Vector2d(0.1, -0.05));

  painter.setPen(pen);
  painter.drawLine(back, front);
  painter.drawLine(left, right);

  drawTriangle(painter, arrLeft, front, arrRight, pen);

  if (withText) {
    QPoint pos = worldToPanel(pose.getPosition());
    painter.drawText(pos.x() + worldToPanel(0.25), pos.y() + 4, getQString(pose));
  }
}

void WorldViewer::drawLineSegment(QPainter& painter,
				  const LineSegment& segment)
{
  Pose2D begin = segment.getStartPose();
  Pose2D end = segment.getEndPose();
  
  QPoint beginLeft = worldToPanel(begin * Vector2d(0, 0.05));
  QPoint beginRight = worldToPanel(begin * Vector2d(0, -0.05));
  QPoint endLeft = worldToPanel(end * Vector2d(0, 0.05));
  QPoint endRight = worldToPanel(end * Vector2d(0, -0.05));

  painter.setPen(_carHelperPen);
  
  // Draw left/right and center line of segment
  if (segment.getExitAngle() == 0) {
    painter.drawLine(worldToPanel(begin.getPosition()), worldToPanel(end.getPosition()));
  } else {
    bool leftCurve = segment.getExitAngle() > 0;
    double radius = segment.getLength() / segment.getExitAngle();
    QPoint origin = worldToPanel(begin * Vector2d(0, radius));
    int iRadius = worldToPanel(radius);
    Angle start = begin.getAngle();
    if (!leftCurve) {
      start = start.opposite();
    }

    painter.drawArc(origin.x() - iRadius,
		    origin.y() - iRadius,
		    2 * iRadius,
		    2 * iRadius,
		    worldToPanel(start),
		    Angle::toDegrees(segment.getExitAngle()) * 16);
  }

  // Draw segment begin- and end-line
  painter.drawLine(beginLeft, beginRight);
  painter.drawLine(endLeft, endRight);
}

void WorldViewer::drawSegment(QPainter& painter,
			      Segment::Ptr segment)
{
  Pose2D begin = segment->getBeginLink()->getPose();
  Pose2D end = Geometry::calculateArcPose(segment->getLength(), segment->getBendingAngle());
  end = begin * end;
  
  QPoint beginLeft = worldToPanel(begin * Vector2d(0, 0.5));
  QPoint beginRight = worldToPanel(begin * Vector2d(0, -0.5));
  QPoint endLeft = worldToPanel(end * Vector2d(0, 0.5));
  QPoint endRight = worldToPanel(end * Vector2d(0, -0.5));

  // Draw left/right and center line of segment
  if (segment->getBendingAngle().rad() == 0) {
    if (segment->consumesDriveInstruction()) {
      QPainterPath path;
      path.moveTo(beginLeft);
      path.lineTo(endLeft);
      path.lineTo(endRight);
      path.lineTo(beginRight);
      painter.fillPath(path, QColor("#ddddff"));
    }

    // Draw Straight segment
    painter.setPen(_borderPen);
    painter.drawLine(beginLeft, endLeft);
    painter.drawLine(beginRight, endRight);

    // Draw center line if the segment is no crossing
    if (!segment->isCrossing()) {
      QPoint beginCenter = worldToPanel(begin.getPosition());
      QPoint endCenter = worldToPanel(end.getPosition());
      painter.setPen(_centerPen);
      painter.drawLine(beginCenter, endCenter);
    }
  } else {
    bool leftCurve = segment->getBendingAngle().rad() > 0;
    double radius = double(segment->getLength() / segment->getBendingAngle().rad());
    QPoint origin = worldToPanel(begin * Vector2d(0, radius));
    int iRadius = worldToPanel(radius);
    int innerRadius = worldToPanel(radius - 0.5);
    int outerRadius = worldToPanel(radius + 0.5);
    Angle start = begin.getAngle();
    if (!leftCurve) {
      start = start.opposite();
    }

    painter.setPen(_centerPen);
    painter.drawArc(origin.x() - iRadius,
		    origin.y() - iRadius,
		    2 * iRadius,
		    2 * iRadius,
		    worldToPanel(start),
		    segment->getBendingAngle().deg() * 16);

    painter.setPen(_borderPen);
    painter.drawArc(origin.x() - innerRadius,
		    origin.y() - innerRadius,
		    2 * innerRadius,
		    2 * innerRadius,
		    worldToPanel(start),
		    segment->getBendingAngle().deg() * 16);
    painter.drawArc(origin.x() - outerRadius,
		    origin.y() - outerRadius,
		    2 * outerRadius,
		    2 * outerRadius,
		    worldToPanel(start),
		    segment->getBendingAngle().deg() * 16);
    
    painter.setPen(_helperLinePen);
    if (leftCurve) {
      painter.drawLine(origin, beginLeft);
      painter.drawLine(origin, endLeft);
    } else {
      painter.drawLine(origin, beginRight);
      painter.drawLine(origin, endRight);
    }
  }

  // Draw Signs belonging to segment 
  vector<RoadSign::Ptr> signs = segment->getRoadSigns();
  for(unsigned int i=0; i < signs.size(); i++) {
	RoadSign::Ptr sign = signs[i];
	
	stringstream fileName;
	fileName << ":/res/" << sign->getName() << ".png";
	string file = fileName.str();
	QImage image(file.c_str());

	QPoint top(-0.8 * _scale, 0.8 * _scale);

	Angle signAngle = sign->getPose().getAngle();
	signAngle -= Angle::deg(90);
	image = image.transformed(QTransform().rotate(signAngle.deg()));
	
	QRect target;
	// sign in segment
	QPoint end = beginLeft - top;
	target.setBottomLeft(beginLeft);
	target.setTopRight(end);

	painter.setOpacity(0.3);
	painter.drawImage(target, image, image.rect());
  	painter.setOpacity(1.0);
  }


  // Draw segment begin- and end-line
  painter.setPen(_borderPen);
  painter.drawLine(beginLeft, beginRight);
  painter.drawLine(endLeft, endRight);

  // Draw segment links and subsequent segments
  if (segment) {
    drawSegmentLink(painter, segment->getBeginLink());

    if (segment->hasStraightOption()) {
      if (segment->getStraightOption()->hasSegmentAfter()) {
	drawSegment(painter, segment->getStraightOption()->getSegmentAfter());
      } else {
	drawSegmentLink(painter, segment->getStraightOption());
      }
    }

    if (segment->hasLeftOption()) {
      if (segment->getLeftOption()->hasSegmentAfter()) {
	drawSegment(painter, segment->getLeftOption()->getSegmentAfter());
      } else {
	drawSegmentLink(painter, segment->getLeftOption());
      }
    }

    if (segment->hasRightOption()) {
      if (segment->getRightOption()->hasSegmentAfter()) {
	drawSegment(painter, segment->getRightOption()->getSegmentAfter());
      } else {
	drawSegmentLink(painter, segment->getRightOption());
      }
    }
  }
}

void WorldViewer::drawTrail(QPainter& painter)
{
  vector<Vector2d> trailPoints = _worldModel->getVirtualTrail();
  painter.setPen(_trailPen);

  for(Vector2d &point : trailPoints)
  {
    QPointF qPoint = worldToPanel(point);
    painter.drawPoint(qPoint);
  }
}

void WorldViewer::drawRoadSigns(QPainter& painter)
{
	vector<RoadSign::Ptr> signs = _worldModel->getRoadSigns();
	
	for(RoadSign::Ptr sign : signs) {
		string signName = sign->getName();
		
		stringstream fileName;
		fileName << ":/res/" << signName << ".png";
		string file = fileName.str();

		QPoint top(-0.5 * _scale, 0.5 * _scale);

	
		// sign in world
		Pose2D signPose = sign->getPose();
		QPoint position = worldToPanel(signPose.x(), signPose.y());
		QPoint end = position - top;

		QRect target;
		target.setBottomLeft(position);
		target.setTopRight(end);
		painter.setPen(Qt::black);

		if(_drawSignImages) {
			QImage image(file.c_str());
			Angle signAngle = sign->getPose().getAngle();
			signAngle -= Angle::deg(90);
			image = image.transformed(QTransform().rotate(signAngle.deg()));
			painter.drawImage(target, image, image.rect());
			painter.drawText(position, QString::number(sign->getSeenCount()));
		}
		else {
			painter.drawText(position, QString::number(sign->getSeenCount()) + " " + QString::fromStdString(signName));
		}
		
	}
}

void WorldViewer::drawSegmentLink(QPainter& painter,
				  SegmentLink::Ptr link)
{
  bool intendedOption = link->isIntendedOption();
  double size = intendedOption ? 0.13 : 0.1;
  Pose2D linkPose = link->getPose();
  QPoint leftInner = worldToPanel(linkPose * Vector2d(0, size));
  QPoint rightInner = worldToPanel(linkPose * Vector2d(0, -size));
  QPoint front = worldToPanel(linkPose * Vector2d(size, 0));

  if (intendedOption) {
    drawTriangle(painter, leftInner, rightInner, front, _intendedLinkPen);
  } else {
    drawTriangle(painter, leftInner, rightInner, front, _linkPen);
  }
  drawPose(painter, link->getGlobalSupportPose(), _supportPosePen);
}

QPoint WorldViewer::worldToPanel(const double& x,
				 const double& y)
{
  return QPoint(int((_yShift - y) * _scale), int((_xShift - x) * _scale));
}

QPoint WorldViewer::worldToPanel(const Vector2d& pos)
{
  return QPoint(int((_yShift - pos(1)) * _scale), int((_xShift - pos(0)) * _scale));
}

int WorldViewer::worldToPanel(const double& length)
{
  return int(length * _scale);
}

int WorldViewer::worldToPanel(const Angle& angle)
{
  if (angle.rad() < 0) {
    return int((angle.deg() + 360) * 16);
  } else {
    return int(angle.deg() * 16);
  }
}

QPointF WorldViewer::panelToWorld(const QPoint& pos)
{
  return QPointF(float(_xShift - ((pos.y()- _halfHeight) / _scale)), float(_yShift - ((pos.x()- _halfWidth) / _scale)));
}

QPointF WorldViewer::panelToWorld(const int& x, const int& y)
{
  return QPointF(float(_xShift - ((y - _halfHeight) / _scale)), float(_yShift - ((x - _halfWidth) / _scale)));
}

QString WorldViewer::getQString(const Pose2D& pose)
{
  stringstream ss;
  ss << std::fixed << std::setprecision(3) << "(" << pose.x() << " | " << pose.y() << " | " << std::setprecision(1) << pose.getAngle().deg() << ")";
  
  return QString(ss.str().c_str());
}

QString WorldViewer::getQString(const double& value,
				const int& precision)
{
  stringstream ss;
  ss << std::fixed << std::setprecision(precision) << value;
  
  return QString(ss.str().c_str());
}

QString WorldViewer::getQString(const int& value)
{
  stringstream ss;
  ss << value;
  
  return QString(ss.str().c_str());
}

void WorldViewer::drawTriangle(QPainter& painter,
			       const QPoint& p1,
			       const QPoint& p2,
			       const QPoint& p3,
			       const QPen& pen)
{
  QPainterPath path;
  path.moveTo(p1);
  path.lineTo(p2);
  path.lineTo(p3);
  
  painter.fillPath(path, pen.brush());
}

void WorldViewer::drawPolygon(QPainter& painter,
			      Polygon::Ptr poly,
			      const QPen& borderPen)
{
  if (!poly || poly->getPoints().size() < 3) {
    return;
  }
  
  QPainterPath path;
  extractPath(poly->getPoints(), path);
  
  painter.setPen(borderPen);
  painter.drawPath(path);
}

void WorldViewer::drawAndFillPolygon(QPainter& painter,
				     Polygon::Ptr poly,
				     const QPen& borderPen,
				     const QPen& fillPen)
{
  if (!poly || poly->getPoints().size() < 3) {
    return;
  }
  
  QPainterPath path;
  extractPath(poly->getPoints(), path);
  
  painter.setPen(borderPen);
  painter.fillPath(path, fillPen.brush());
  painter.drawPath(path);
}

void WorldViewer::extractPath(const vector<Vector2d>& points,
			      QPainterPath& path,
			      const bool& closePath)
{
  if (points.size() < 2) {
    return;
  }
  
  QPoint startPoint = worldToPanel(points[0]);
  path.moveTo(startPoint);
  for (Vector2d p : points) {
    path.lineTo(worldToPanel(p));
  }
  if (closePath) {
    path.lineTo(startPoint);
  }
}

void WorldViewer::drawObjects(QPainter& painter)
{
  std::vector<IVisibleObject::Ptr> objects = _worldModel->getObjects();
  for(IVisibleObject::Ptr object : objects)
  {
    if(object)
    {
    	drawPose(painter, object->getPose(), QPen(QColor("#0600ff")));
   	Polygon::ConstPtr poly = object->getBoundingPoly();
    	Pose2D pose = object->getPose();
    	Polygon::Ptr polyWorld =  pose * poly;
    	drawPolygon(painter, polyWorld, QPen(QColor("#0600ff")));
    }

  }
  
}
