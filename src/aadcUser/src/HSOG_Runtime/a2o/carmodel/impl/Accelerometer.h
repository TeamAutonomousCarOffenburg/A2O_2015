#pragma once

#include "Sensor.h"
#include "carmodel/IAccelerometer.h"
#include "meta/ISensorConfig.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The Accelerometer class represents an accelerometer sensor.
 * 
 * \author Stefan Glaser
 */
class Accelerometer : public Sensor, public virtual IAccelerometer {
public:
  typedef boost::shared_ptr<Accelerometer> Ptr;
  typedef boost::shared_ptr<const Accelerometer> ConstPtr;

  Accelerometer(ISensorConfig::ConstPtr config);
  virtual ~Accelerometer();
  
  virtual const Eigen::Vector3d& getAcceleration() const;
  
  virtual const bool update(IPerception::ConstPtr perception);
  
protected:
  Eigen::Vector3d _acc;
};

}
