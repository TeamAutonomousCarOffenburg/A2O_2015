#pragma once

#include "Maneuver.h"


namespace A2O {

class IsReadyManeuver : public Maneuver {
public:
  IsReadyManeuver(ICarModel::Ptr carModel,
		   IWorldModel::Ptr worldModel);
  virtual ~IsReadyManeuver();
  
  virtual void perform();
  virtual const ManeuverStatus getStatus() const override;
};

}
