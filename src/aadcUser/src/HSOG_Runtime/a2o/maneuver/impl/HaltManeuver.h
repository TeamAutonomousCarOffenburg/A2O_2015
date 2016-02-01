#pragma once

#include "Maneuver.h"


namespace A2O {

/**
 * This maneuver stops the car and indicates a hazard situation.
 * While this maneuver is similar to an 'emergency stop' in terms of car driver language,
 * it does NOT check any surrounding conditions and maintains the current steering angle.
 * It is therefore more like a 'emergency stop' or 'system halt' from an engineer's perspective.
 * 
 * \author Stefan Glaser
 */
class HaltManeuver : public Maneuver {
public:
  HaltManeuver(ICarModel::Ptr carModel,
	       IWorldModel::Ptr worldModel);
  virtual ~HaltManeuver();
  
  virtual void perform();
};

}
