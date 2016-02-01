#pragma once

#include "Maneuver.h"
#include "utils/geometry/Pose2D.h"

namespace A2O {

/**
 * This maneuver drives the car for one meter.
 * 
 * \author Stefan Glaser
 */
class BlindParking : public Maneuver {
public:
  BlindParking(ICarModel::Ptr carModel,
	       IWorldModel::Ptr worldModel);
  virtual ~BlindParking();
  
  virtual void perform();

private:
  Pose2D _startPose;

};

}
