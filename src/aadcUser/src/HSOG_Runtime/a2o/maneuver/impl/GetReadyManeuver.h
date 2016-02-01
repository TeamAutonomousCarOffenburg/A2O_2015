#pragma once

#include "Maneuver.h"


namespace A2O {

/**
 * This maneuver is a kind of pausing maneuver that is
 * performed during the setup phase before a test run.
 * 
 * \author Stefan Glaser
 */
class GetReadyManeuver : public Maneuver {
public:
  GetReadyManeuver(ICarModel::Ptr carModel,
		   IWorldModel::Ptr worldModel);
  virtual ~GetReadyManeuver();
  
  virtual void perform();
  virtual const ManeuverStatus getStatus() const override;
};

}
