#pragma once

#include "IWorldRenderer.h"


/**
 * The GridRenderer renders a 1 by 1 meter grid.
 *
 * \author Stefan Glaser
 */
class GridRenderer : public virtual IWorldRenderer {
public:
  GridRenderer();
  ~GridRenderer();
  
  virtual void render(A2O::IWorldModel::Ptr worldModel,
		      A2O::ICarModel::Ptr carModel,
		      QPainter& painter,
		      const PanelTransformation& panelTransform);
};
