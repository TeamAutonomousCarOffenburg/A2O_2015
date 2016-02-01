#pragma once

#include "SegmentDetection.h"
#include <utils/geometry/Circle2D.h>


namespace A2O {

class Curve2Detection : public virtual SegmentDetection {
public:
  Curve2Detection(const Circle2D& circle,
		 const bool& leftCurve);
  virtual ~Curve2Detection();
  
  const Circle2D& getCircle() const;
  const bool& isLeftCurve() const;
  
  void add(const Circle2D& circle);
  
  const double getOriginDistanceTo(const Circle2D& circle) const;
  const double getAbsRadiusDistanceTo(const Circle2D& circle) const;
  
private:
  /** The average circle that corresponds to this curve detection. */
  Circle2D _circle;
  
  /** Indicator if this curve detection corresponds to a left (true) or right (false) curve. */
  bool _leftCurve;
};

}
