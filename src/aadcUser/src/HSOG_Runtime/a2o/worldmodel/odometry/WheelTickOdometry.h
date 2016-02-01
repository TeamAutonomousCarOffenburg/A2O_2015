#pragma once

#include "utils/geometry/Pose2D.h"
#include "utils/geometry/drive/DriveCalculator.h"
#include "carmodel/ICarModel.h"
#include <queue>


namespace A2O {

class WheelTickOdometry {
public:
  WheelTickOdometry();
  WheelTickOdometry(DriveCalculator::ConstPtr driveCalculator, const double& wheelDiameter);
  ~WheelTickOdometry();
  
  void setDriveCalculator(DriveCalculator::ConstPtr driveCalculator);
  
  const Pose2D& getCurrentPose();
  
  const Pose2D update(const double& leftWheelTicks,
		      const double& rightWheelTicks,
		      const long& time);
  
  void reset(const Pose2D& zeroPose,
	     const long& time);
  
private:
  const double getRPMS(const std::pair<long, double>& tm1,
		       const std::pair<long, double>& tm2);
  
  Pose2D _pose;
  
  long _time;
  
  DriveCalculator::ConstPtr _driveCalculator;
  
  const double _wheelDiameter;
  
  std::deque<std::pair<long, double>> _leftTickHistory;
  
  std::deque<std::pair<long, double>> _rightTickHistory;
  
};

}
