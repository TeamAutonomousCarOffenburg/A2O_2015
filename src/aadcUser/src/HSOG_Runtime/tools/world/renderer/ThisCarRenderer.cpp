#include "ThisCarRenderer.h"

#include "tutils/RenderUtil.h"
#include "worldmodel/impl/ThisCar.h"


using namespace A2O;
using namespace Eigen;


ThisCarRenderer::ThisCarRenderer()
{
  _carPen = QPen(Qt::red);
  _carPen.setWidthF(0.02);
  
  _carBackgroundPen.setColor(QColor(255, 0, 0, 128));
  
  _carHelperPen = QPen(Qt::DashLine);
  _carHelperPen.setColor(Qt::red);
  _carHelperPen.setWidthF(0.015);
  
  _wayPointPen = QPen(QColor("#00bb00"));
  _wayPointPen.setWidthF(0.01);
  
  _wayPen = QPen(Qt::DashLine);
  _wayPen.setColor(QColor("#00bb00"));
  _wayPen.setWidthF(0.02);
}

ThisCarRenderer::~ThisCarRenderer()
{
}

void ThisCarRenderer::render(IWorldModel::Ptr worldModel,
			     ICarModel::Ptr carModel,
			     QPainter& painter,
			     const PanelTransformation& panelTransform)
{
  IThisCar::Ptr thisCar = worldModel->getThisCar();
  
  // Draw Path
  Path2D::ConstPtr path = thisCar->getPath();
  std::vector<Pose2D> wayPoints = path->getWayPoints();
  painter.setPen(_wayPointPen);
  for (Pose2D wayPoint : wayPoints) {
    panelTransform.setTransform(wayPoint, painter);
    RenderUtil::drawPoseArrow(painter, _wayPointPen.brush());
  }
  std::vector<LineSegment> lineSegments = path->getLineSegments();
  for (LineSegment segment : lineSegments) {
    drawSegment(painter, segment, panelTransform);
  }
  
  // Draw car
  Pose2D carPose = thisCar->getPose();
  panelTransform.setTransform(carPose, painter);
  
  painter.setPen(_carPen);
  RenderUtil::fillAndDrawPolygon(painter, thisCar->getBoundingPoly()->getPoints(), _carBackgroundPen.brush());
  RenderUtil::drawPoseArrow(painter, _carPen.brush());
}

void ThisCarRenderer::drawSegment(QPainter& painter,
				  const LineSegment& segment,
				  const PanelTransformation& panelTransform)
{
  painter.setPen(_wayPointPen);
  panelTransform.setTransform(segment.getStartPose(), painter);
  painter.drawLine(QPointF(0, 0.05), QPointF(0, -0.05));
  
  painter.setPen(_wayPen);
  RenderUtil::drawLineSegment(painter, segment.getLength(), segment.getExitAngle());
  
  painter.setPen(_wayPointPen);
  panelTransform.setTransform(segment.getEndPose(), painter);
  painter.drawLine(QPointF(0, 0.05), QPointF(0, -0.05));
}
