#pragma once

#include "IWorldRenderer.h"


/**
 * The StreetRenderer renders the tree of street segments.
 *
 * \author Stefan Glaser
 */
class StreetRenderer : public virtual IWorldRenderer {
public:
  StreetRenderer();
  ~StreetRenderer();
  
  virtual void render(A2O::IWorldModel::Ptr worldModel,
		      A2O::ICarModel::Ptr carModel,
		      QPainter& painter,
		      const PanelTransformation& panelTransform);
  
  void drawSegment(QPainter& painter,
		   A2O::Segment::Ptr segment,
		   const PanelTransformation& panelTransform);
  void drawSubSegments(QPainter& painter,
		       A2O::Segment::Ptr segment,
		       const PanelTransformation& panelTransform);
  void drawSegmentLink(QPainter& painter,
		       A2O::SegmentLink::Ptr segmentLink,
		       const PanelTransformation& panelTransform);
  
private:
  QPen _streetPen;
  QPen _thinStreetPen;
  QPen _instructionStreetPen;
  QPen _streetLinePen;
  QPen _stopLinePen;
  QPen _middleLinePen;
  QPen _helperLinePen;
  
  QPen _linkPen;
  QPen _intendedLinkPen;
  QPen _supportPosePen;
};
