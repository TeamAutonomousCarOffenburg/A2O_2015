#pragma once

#include "IWorldRenderer.h"


/**
 * The ThisCarRenderer renders all relevant information of our own car.
 *
 * \author Stefan Glaser
 */
class ThisCarRenderer : public virtual IWorldRenderer {
public:
  ThisCarRenderer();
  ~ThisCarRenderer();
  
  virtual void render(A2O::IWorldModel::Ptr worldModel,
		      A2O::ICarModel::Ptr carModel,
		      QPainter& painter,
		      const PanelTransformation& panelTransform);
  
  void drawSegment(QPainter& painter,
		   const A2O::LineSegment& segment,
		   const PanelTransformation& panelTransform);
private:
  QPen _carPen;
  QPen _carBackgroundPen;
  QPen _carHelperPen;
  QPen _wayPointPen;
  QPen _wayPen;
};
