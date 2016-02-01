#pragma once

#include "SegmentDetection.h"
#include <utils/geometry/Pose2D.h>


namespace A2O {
  
enum CrossingType {
  T_LS_CROSSING,
  T_LR_CROSSING,
  T_SR_CROSSING,
  X_CROSSING
};

class CrossingDetection : public virtual SegmentDetection {
public:
  CrossingDetection(const Pose2D& pose,
		    const CrossingType& type);
  virtual ~CrossingDetection();
  
  const Pose2D& getPose() const;
  const CrossingType& getMostLikelyType() const;
  
  void add(const Pose2D& pose,
	   const CrossingType& type);
  
  const double getPositionDistanceTo(const Pose2D& pose) const;
  const double getAngularDistanceTo(const Pose2D& pose) const;
  
private:
  void incrementType(const CrossingType& type);
  
  /** The average pose that corresponds to the crossing detection. */
  Pose2D _pose;
  
  /** The most likely crossing type. */
  CrossingType _mostLikelyType;
  
  /** Individual occurence counter for T-left-straight crossings. */
  float _TLSCount;
  
  /** Individual occurence counter for T-left-right crossings. */
  float _TLRCount;
  
  /** Individual occurence counter for T-straight-right crossings. */
  float _TSRCount;
  
  /** Individual occurence counter for X crossings. */
  float _XCount;
};

}
