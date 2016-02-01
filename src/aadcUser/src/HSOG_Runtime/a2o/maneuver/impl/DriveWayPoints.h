#pragma once

#include "Maneuver.h"
#include "utils/geometry/Line2D.h"
#include "utils/geometry/Angle.h"
#include "utils/geometry/Circle2D.h"
#include "utils/geometry/Cosine.h"
#include "utils/geometry/drive/DriveGeometry.h"


namespace A2O {
  
 /** 
  * This maneuver tries to follow the way points by a possible geometry;
  * 
  * \author Peter Walden, Stefan Glaser
  */

class DriveWayPoints : public virtual Maneuver {
public:
  DriveWayPoints(ICarModel::Ptr carModel,
		 IWorldModel::Ptr worldModel);
  ~DriveWayPoints();

  virtual void perform();
  
  const Pose2D getNextWayPoint(int skip = 0) const;

private:
  void driveWayPoint(int skip = 0);
  void followGeometry(IManeuverGeometry* geometry, Pose2D& carPose, Pose2D& nextWayPoint);
  Angle getSteeringAngle(IManeuverGeometry* geometry, Pose2D& carPose);

  const double _speed = 30;  

};

}
