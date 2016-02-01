#pragma once

#include "Maneuver.h"
#include "utils/geometry/Pose2D.h"
#include "utils/geometry/Line2D.h"
#include "utils/geometry/Circle2D.h"
#include "utils/geometry/IManeuverGeometry.h"
#include <math.h>
#include "AADCCar.h"

namespace A2O {

/**
 * This maneuver follows a virtual drive lane, e.g. accross a crossing.
 * 
 * \author Stefan Glaser, Peter Walden
 */
class DriveVirtualLane : public Maneuver {
public:
  DriveVirtualLane(ICarModel::Ptr carModel,
		   IWorldModel::Ptr worldModel);
  virtual ~DriveVirtualLane();
  
  virtual void perform();
  virtual const ManeuverStatus getStatus() const override;
  
//   static void getAngle(Pose2D& currentPose, IManeuverGeometry* geometry, Angle& result);
  
private:
//  virtual void calcVitualLane(Pose2D* fromPose, Pose2D* toPose);
};

}
