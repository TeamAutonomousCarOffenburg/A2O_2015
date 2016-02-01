#pragma once

#include "SegmentDetection.h"
#include <utils/geometry/Pose2D.h>


namespace A2O {

class StraightDetection : public virtual SegmentDetection {
public:
  StraightDetection(const Pose2D& pose);
  virtual ~StraightDetection();
  
  const Pose2D& getPose() const;
  
  void add(const Pose2D& pose);
  
  const double getPositionDistanceTo(const Pose2D& pose) const;
  const double getAngularDistanceTo(const Pose2D& pose) const;
  
private:
  Pose2D _pose;
};

}
