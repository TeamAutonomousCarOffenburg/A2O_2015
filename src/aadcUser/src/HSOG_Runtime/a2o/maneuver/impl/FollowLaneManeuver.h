#pragma once

#include "Maneuver.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include "AADCCar.h"

namespace A2O {

/**
 * This maneuver tries to follow the lane of the real road by navigating along it.
 * 
 * \author Stefan Glaser
 */
class FollowLaneManeuver : public Maneuver {
public:
  FollowLaneManeuver(ICarModel::Ptr carModel,
		     IWorldModel::Ptr worldModel);
  virtual ~FollowLaneManeuver();
  
  virtual void perform();
  
private:
  // in millimeter
  const double laneHalfWidth = 250;

};

}
