#pragma once

#include "ISensor.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for a Accelerometer sensor.
 * 
 * \author Stefan Glaser
 */
class IAccelerometer : public virtual ISensor {
public:
  typedef boost::shared_ptr<IAccelerometer> Ptr;
  typedef boost::shared_ptr<const IAccelerometer> ConstPtr;

  virtual ~IAccelerometer(){};
  
  /***/
  virtual const Eigen::Vector3d& getAcceleration() const = 0;
};

}
