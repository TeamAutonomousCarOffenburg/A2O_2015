#pragma once

#include "Maneuver.h"
#include "utils/geometry/Pose2D.h"

namespace A2O {

/**
 * This maneuver drives the car for one meter.
 * 
 * \author Stefan Glaser
 */
class OneMeter : public Maneuver {
public:
  OneMeter(ICarModel::Ptr carModel,
	       IWorldModel::Ptr worldModel);
  virtual ~OneMeter();
  
  virtual void perform();

private:
  Pose2D _startPose;

};

}
