#pragma once

#include "Maneuver.h"


namespace A2O {

class DriveWayPointsStraight : public virtual Maneuver {
public:
  DriveWayPointsStraight(ICarModel::Ptr carModel,
		 IWorldModel::Ptr worldModel);
  DriveWayPointsStraight();

  virtual void perform();
  
  const Pose2D getNextWayPoint() const;

private:
  
};

}
