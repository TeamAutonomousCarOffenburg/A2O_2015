#pragma once

#include "Sensor.h"
#include "carmodel/IGyroscope.h"
#include "meta/ISensorConfig.h"
#include "utils/value/Distribution.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The Gyroscope class represents an gyroscope sensor.
 * 
 * \author Stefan Glaser
 */
class Gyroscope : public Sensor, public virtual IGyroscope {
public:
  typedef boost::shared_ptr<Gyroscope> Ptr;
  typedef boost::shared_ptr<const Gyroscope> ConstPtr;

  Gyroscope(ISensorConfig::ConstPtr config);
  virtual ~Gyroscope();
  
  virtual const Eigen::Quaterniond& getGyro() const;
  virtual const Angle getHorizontalAngle() const;
  
  virtual const bool update(IPerception::ConstPtr perception);
  virtual const bool isInitialized() const;
  virtual void reset();
  
protected:
  Eigen::Quaterniond _gyro;
  
  Eigen::Quaterniond _orientation;
  
  Eigen::Quaterniond _initialOrientation;
  
  Distribution* _distribution;
};

}
