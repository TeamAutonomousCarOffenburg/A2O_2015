#pragma once

#include "tutils/PanelTransformation.h"

#include <QtCore/QtCore>
#include <QtGui/QtGui>

#include <worldmodel/IWorldModel.h>
#include <carmodel/ICarModel.h>


/**
 * The IWorldRenderer represents the basic interface for all renderer within our world-viewer.
 *
 * \author Stefan Glaser
 */
class IWorldRenderer {
public:
  virtual ~IWorldRenderer(){};
  
  virtual void render(A2O::IWorldModel::Ptr worldModel,
		      A2O::ICarModel::Ptr carModel,
		      QPainter& painter,
		      const PanelTransformation& panelTransform) = 0;
};
