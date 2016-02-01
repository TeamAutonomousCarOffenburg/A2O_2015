#pragma once

#include "utils/geometry/Angle.h"
#include "utils/geometry/Pose2D.h"


namespace A2O {

/** Class for odomotry calculation based on gyroscope sensor and distance travelled
  * measured by wheel tick sensors.
  *
  * \author Stefan Glaser
  */
class GyroOdometry {
public:
  GyroOdometry(const double& wheelDiameter,
	       const int& ticksPerRound);
  ~GyroOdometry();
  
  void init(const double& ticks,
	    const Angle& zAngle);
  
  void reset(const Pose2D& pose);

  const Pose2D& getPose() const;

  const Pose2D& update(const double& ticks,
		       const Angle& zAngle);

private:
  const double _wheelDiameter;
  
  const int _ticksPerRound;

  Pose2D _pose;

  double _previousTicks;

  Angle _previousAngle;
};

}
