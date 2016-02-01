#pragma once

#include "carmodel/ICarModel.h"
#include "utils/geometry/Pose2D.h"

#define CORRECTION_FACTOR 0.87
#define ANGLE_REAR_WHEELS 4.5
#define REAR_STEERING_LOWER_THRESHOLD 10
#define REAR_STEERING_UPPER_THRESHOLD 20


namespace A2O {

/** Class for odomotry calculation based on steering angle sensor and distance travelled 
  * measured by wheel tick sensors  
  *
  * \author Peter Walden
  */

// not working, yet
class SteeringAngleOdometry {
public:
  SteeringAngleOdometry(): _wheelDiameter(0.1), _frontAxlePosition(Eigen::Vector3d(.3,0,0)),  
      _frontAxleHalfLength(0.24 * 0.5), _rearAxleHalfLength(0.26 * 0.5)
  { _wheelTickOffset = 0;};
  SteeringAngleOdometry(const double& wheelDiameter, const Eigen::Vector3d& frontAxlePosition, const double& frontAxleLength, 
			const double& rearAxleLength);
  ~SteeringAngleOdometry(){};
  Pose2D update(const double& wheelTicks, const double& steeringAngle);
  Pose2D& getCurrentPose();
  void setCurrentPose(const Pose2D& pose);
  
private:
  const double _wheelDiameter;
  const Eigen::Vector3d _frontAxlePosition;
  const double _frontAxleHalfLength;
  const double _rearAxleHalfLength;
  Pose2D _pose;
  double _wheelTickOffset;

};
}