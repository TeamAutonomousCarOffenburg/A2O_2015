#pragma once

#include "ISensor.h"
#include "utils/geometry/Angle.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for a Gyroscope sensor.
 * 
 * \author Stefan Glaser
 */
class IGyroscope : public virtual ISensor {
public:
  typedef boost::shared_ptr<IGyroscope> Ptr;
  typedef boost::shared_ptr<const IGyroscope> ConstPtr;

  virtual ~IGyroscope(){};
  
  /***/
  virtual const Eigen::Quaterniond& getGyro() const = 0;
  
  /***/
  virtual const Angle getHorizontalAngle() const = 0;
  
  virtual void reset() = 0;
};

}
