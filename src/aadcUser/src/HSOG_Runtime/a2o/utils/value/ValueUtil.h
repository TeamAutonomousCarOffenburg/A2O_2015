#pragma once

#include "utils/geometry/Angle.h"


namespace A2O {

/** The ValueUtil class provides different value calculations.
 *
 * \author Stefan Glaser
 */
class ValueUtil {
public:
  /** Limit the given value within the range +/- limit. */
  template<typename T>
  static void limitAbs(T& value, const T& limit);
  
  /** Limit the given value within the given bounds [lowerBound, upperBound]. */
  template<typename T>
  static void limit(T& value, const T& lowerBound, const T& upperBound);
  
  /** Retrieve a copy of value that is ensured to be within the range +/- limit. */
  template<typename T>
  static const T valueInAbsLimit(const T& value, const T& limit);
  
  /** Retrieve a copy of value that is ensured to be within the bounds [lowerBound, upperBound]. */
  template<typename T>
  static const T valueInLimit(const T& value, const T& lowerBound, const T& upperBound);
};

}
