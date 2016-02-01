#pragma once

#include "ISensor.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Interface for a distance sensor.
 * 
 * \author Stefan Glaser
 */
class IDistanceSensor : public virtual ISensor {
public:
  typedef boost::shared_ptr<IDistanceSensor> Ptr;
  typedef boost::shared_ptr<const IDistanceSensor> ConstPtr;

  virtual ~IDistanceSensor(){};
  
  /** Retrieve the minimum distance this sensor can measure.
   * \returns the minimum distance
   */
  virtual const double& getMinDistance() const = 0;
  
  
  /** Retrieve the maximum distance this sensor can measure.
   * \returns the maximum distance
   */
  virtual const double& getMaxDistance() const = 0;
  
  
  /** Retrieve the width of the distance scan.
   * \returns the scan width
   */
  virtual const double& getScanWidth() const = 0;
  
  /** Retrieve the most recent distance measurement.
   * 
   * \returns the measured distance
   */
  virtual const double& getDistance() const = 0;
  
  /** Retrieve the most recent distance measurement, limited by the min and max distance.
   * 
   * \returns the limited measured distance
   */
  virtual const double& getLimitedDistance() const = 0;
  
  /** Check if the current distance value is within the valid range of this sensor.
    * \returns true, if the currently measured distance is within the valid range, false otherwise
    */
  virtual const bool inRange() const = 0;
  
  /** Compute the percent of the current measurement with respect to the sensor range.
    * 0 percent --> distance == minDistance; 100 percent --> distance == maxDistance
    * \returns the percent (current distance in range)
    */
  virtual const double getPercent() const = 0;
};

}
