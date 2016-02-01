#include "StreetRenderer.h"

#include "tutils/RenderUtil.h"
#include "utils/geometry/Geometry.h"
#include <worldmodel/IDriveInstructionConstants.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

using namespace A2O;
using namespace Eigen;


StreetRenderer::StreetRenderer()
{
  _streetPen = QPen(QColor("#333333"));
  _streetPen.setCapStyle(Qt::FlatCap);
  _streetPen.setWidthF(1.0);
  _thinStreetPen = QPen(QColor("#333333"));
  _thinStreetPen.setCapStyle(Qt::FlatCap);
  _thinStreetPen.setWidthF(0.07);
  
  _instructionStreetPen = QPen(QColor("#444466"));
  _instructionStreetPen.setCapStyle(Qt::FlatCap);
  _instructionStreetPen.setWidthF(1.0);
  
  _streetLinePen = QPen(Qt::white);
  _streetLinePen.setCapStyle(Qt::RoundCap);
  _streetLinePen.setWidthF(0.03);
  _stopLinePen = QPen(Qt::white);
  _stopLinePen.setWidthF(0.05);
  _middleLinePen = QPen(Qt::DashLine);
  _middleLinePen.setWidthF(0.02);
  QVector<qreal> dashes;
  dashes << 0 << 5 << 14.75 << 5;
  _middleLinePen.setDashPattern(dashes);
  _middleLinePen.setColor(Qt::white);
  _middleLinePen.setCapStyle(Qt::FlatCap);
  
  _helperLinePen = QPen(Qt::DashLine);
  _helperLinePen.setColor(QColor("#999999"));
  _helperLinePen.setWidthF(0.02);
  
  
  _linkPen = QPen(Qt::blue);
  _linkPen.setWidthF(0.005);
  _linkPen.setCapStyle(Qt::RoundCap);
  _intendedLinkPen = QPen(Qt::darkGreen);
  _intendedLinkPen.setWidthF(0.005);
  _intendedLinkPen.setCapStyle(Qt::RoundCap);
  _supportPosePen = QPen(Qt::lightGray);
  _supportPosePen.setWidthF(0.01);
}

StreetRenderer::~StreetRenderer()
{
}

void StreetRenderer::render(A2O::IWorldModel::Ptr worldModel,
			    A2O::ICarModel::Ptr carModel,
			    QPainter& painter,
			    const PanelTransformation& panelTransform)
{
  Segment::Ptr seg = worldModel->getCurrentWorldState()->getCurrentSegment();
  
  if (seg) {
    drawSegment(painter, seg, panelTransform);
  }
  
  /*
  Segment::Ptr seg = Segment::createInitialSegment(Pose2D(0, 0, Angle::deg(0)), 1);
 
  std::vector<RoadSign::Ptr> signs;
  signs.push_back(boost::make_shared<RoadSign>(Stop, Pose2D(1,1,Angle::deg(180))));
  seg->attachRoadSigns(signs);

  Segment::Ptr newSeg = Segment::appendSegment(seg->getStraightOption(), 2);
  newSeg = Segment::appendXCrossing(newSeg->getStraightOption());
  newSeg = Segment::appendCurve2MLeft(newSeg->getStraightOption()->getSegmentAfter()->getStraightOption());
  newSeg = Segment::appendXCrossing(newSeg->getStraightOption());
  newSeg = Segment::appendSegment(newSeg->getRightOption()->getSegmentAfter()->getStraightOption(), 2, Angle::deg(-40));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), 1.5, Angle::deg(-110));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), 0.8, Angle::deg(60));
  newSeg = Segment::appendTCrossingLR(newSeg->getStraightOption());
  
  std::vector<std::string> driveInstructions;
  driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_STRAIGHT);
  driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_RIGHT);
  driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_RIGHT);
  DriveInstructionManager::Ptr mngr(boost::make_shared<DriveInstructionManager>(driveInstructions));
  seg->update(mngr, mngr->getCurrentInstructionIndex());
  
  drawSegment(painter, seg, panelTransform);*/
}

void StreetRenderer::drawSegment(QPainter& painter,
				 A2O::Segment::Ptr segment,
				 const PanelTransformation& panelTransform)
{
  Pose2D beginPose = segment->getBeginLink()->getPose();
  Pose2D endPose = Geometry::calculateArcPose(segment->getLength(), segment->getBendingAngle());
  Angle bendingAngle = segment->getBendingAngle();
  bool straightSegment = segment->getBendingAngle().rad() == 0;
  double radius = 0;
  
  // Set the current painting system to the begin of the segment
  panelTransform.setTransform(beginPose, painter);
  
  QPointF begin(0, 0);
  QPointF end(endPose.getPosition()(0), endPose.getPosition()(1));

  // Draw segment background
  if (straightSegment) {
    // Draw street background
    if (segment->consumesDriveInstruction()) {
      painter.setPen(_instructionStreetPen);
    } else {
      painter.setPen(_streetPen);
    }
    painter.drawLine(begin, end);
  } else {
    radius = double(segment->getLength() / bendingAngle.rad());
    // Draw street background
    if (segment->consumesDriveInstruction()) {
      painter.setPen(_instructionStreetPen);
    } else {
      painter.setPen(_streetPen);
    }
    RenderUtil::drawArc(painter, begin, radius, bendingAngle.rad());
    painter.setPen(_thinStreetPen);
    RenderUtil::drawArc(painter, QPointF(0, -0.465), radius + 0.465, bendingAngle.rad());
    RenderUtil::drawArc(painter, QPointF(0, 0.465), radius - 0.465, bendingAngle.rad());
  }
  

  // Draw Signs belonging to segment 
  std::vector<RoadSign::Ptr> signs = segment->getRoadSigns();
  for(unsigned int i=0; i < signs.size(); i++) {
    RoadSign::Ptr sign = signs[i];
    
    std::stringstream fileName;
    fileName << ":/res/" << sign->getName() << ".png";
    std::string file = fileName.str();
    QImage image(file.c_str());

    Angle signAngle = sign->getPose().getAngle();
    signAngle -= Angle::deg(90);
    image = image.transformed(QTransform().rotate(signAngle.deg()));
    image = image.mirrored(false, true);


    QRectF rect(0.0, -0.25, 0.5, 0.5);
    
    painter.drawImage(rect, image);
  }

  // Draw begin link and sub segments
  drawSubSegments(painter, segment, panelTransform);
  panelTransform.setTransform(beginPose, painter);
  
  // Draw segment lines
  if (straightSegment) {
    // Draw segment lines
    painter.setPen(_streetLinePen);
    if (segment->isXCrossing()) {
      painter.drawLine(QPointF(0, 0.465), QPointF(0.035, 0.5));
      painter.drawLine(QPointF(0, -0.465), QPointF(0.035, -0.5));
      painter.drawLine(QPointF(1, 0.465), QPointF(0.965, 0.5));
      painter.drawLine(QPointF(1, -0.465), QPointF(0.965, -0.5));
    } else if(segment->isTCrossingLS()) {
      painter.drawLine(QPointF(0, 0.465), QPointF(0.035, 0.5));
      painter.drawLine(QPointF(1, 0.465), QPointF(0.965, 0.5));
      painter.drawLine(QPointF(0, -0.465), QPointF(1, -0.465));
    } else if(segment->isTCrossingLR()) {
      painter.drawLine(QPointF(0, 0.465), QPointF(0.035, 0.5));
      painter.drawLine(QPointF(0, -0.465), QPointF(0.035, -0.5));
      painter.drawLine(QPointF(0.965, 0.5), QPointF(0.965, -0.5));
    } else if(segment->isTCrossingSR()) {
      painter.drawLine(QPointF(0, 0.465), QPointF(1, 0.465));
      painter.drawLine(QPointF(0, -0.465), QPointF(0.035, -0.5));
      painter.drawLine(QPointF(1, -0.465), QPointF(0.965, -0.5));
    } else {
      painter.drawLine(QPointF(0, 0.465), QPointF(segment->getLength(), 0.465));
      painter.drawLine(QPointF(0, -0.465), QPointF(segment->getLength(), -0.465));
    
      // Draw middle line
      painter.setPen(_middleLinePen);
      painter.drawLine(begin, end);
    }
  } else {
    // Draw left and right line
    painter.setPen(_streetLinePen);
    if (radius > 0) {
      RenderUtil::drawArc(painter, QPointF(0, -0.465), radius + 0.465, bendingAngle.rad());
      RenderUtil::drawArc(painter, QPointF(0, 0.465), radius - 0.465, bendingAngle.rad(), _helperLinePen);
    } else {
      RenderUtil::drawArc(painter, QPointF(0, -0.465), radius + 0.465, bendingAngle.rad(), _helperLinePen);
      RenderUtil::drawArc(painter, QPointF(0, 0.465), radius - 0.465, bendingAngle.rad());
    }
    
    // Draw middle line
    painter.setPen(_middleLinePen);
    RenderUtil::drawArc(painter, begin, radius, bendingAngle.rad());
  }
}

void StreetRenderer::drawSubSegments(QPainter& painter,
				     Segment::Ptr segment,
				     const PanelTransformation& panelTransform)
{
  // Draw segment links and subsequent segments
  if (segment) {
    drawSegmentLink(painter, segment->getBeginLink(), panelTransform);

    if (segment->hasStraightOption()) {
      if (segment->getStraightOption()->hasSegmentAfter()) {
	drawSegment(painter, segment->getStraightOption()->getSegmentAfter(), panelTransform);
      } else {
	drawSegmentLink(painter, segment->getStraightOption(), panelTransform);
      }
    }

    if (segment->hasLeftOption()) {
      if (segment->getLeftOption()->hasSegmentAfter()) {
	drawSegment(painter, segment->getLeftOption()->getSegmentAfter(), panelTransform);
      } else {
	drawSegmentLink(painter, segment->getLeftOption(), panelTransform);
      }
    }

    if (segment->hasRightOption()) {
      if (segment->getRightOption()->hasSegmentAfter()) {
	drawSegment(painter, segment->getRightOption()->getSegmentAfter(), panelTransform);
      } else {
	drawSegmentLink(painter, segment->getRightOption(), panelTransform);
      }
    }
  }
}

void StreetRenderer::drawSegmentLink(QPainter& painter,
				     A2O::SegmentLink::Ptr segmentLink,
				     const PanelTransformation& panelTransform)
{
  Pose2D linkPose = segmentLink->getPose();
  bool intendedOption = segmentLink->isIntendedOption();
  
  panelTransform.setTransform(linkPose, painter);
  
  double size = intendedOption ? 0.13 : 0.1;
  QPointF leftInner(0, size);
  QPointF rightInner(0, -size);
  QPointF front(size, 0);

  if (intendedOption) {
    RenderUtil::fillTriangle(painter, leftInner, rightInner, front, _intendedLinkPen.brush());
    painter.setPen(_intendedLinkPen);
  } else {
    RenderUtil::fillTriangle(painter, leftInner, rightInner, front, _linkPen.brush());
    painter.setPen(_linkPen);
  }
  painter.drawLine(QPointF(0, 0.5), QPointF(0, -0.5));
  
  panelTransform.setTransform(segmentLink->getGlobalSupportPose(), painter);
  painter.setPen(_supportPosePen);
  RenderUtil::drawPoseArrow(painter, _supportPosePen.brush());
}
