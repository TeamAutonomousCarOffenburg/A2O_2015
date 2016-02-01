#pragma once

#include "meta/IAxleConfig.h"
#include "carmodel/IAxle.h"
#include "Wheel.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The Axle class represents an axle of the car.
 * 
 * \author Stefan Glaser
 */
class Axle : public virtual IAxle {
public:
  typedef boost::shared_ptr<Axle> Ptr;
  typedef boost::shared_ptr<const Axle> ConstPtr;

  Axle(IAxleConfig::ConstPtr config);
  ~Axle();
  
  virtual const std::string& getName() const;
  virtual const Eigen::Vector3d& getPosition() const;
  virtual const double& getLength() const;
  virtual IWheel::ConstPtr getLeftWheel() const;
  virtual IWheel::ConstPtr getRightWheel() const;
  
protected:
  /** The name of the axle. */
  const std::string _name;
  
  /** THe axle position. */
  const Eigen::Vector3d _position;
  
  /** The length of the axle. */
  const double _length;
  
  /** The left wheel mounted to this axle. */
  Wheel::Ptr _leftWheel;
  
  /** The left wheel mounted to this axle. */
  Wheel::Ptr _rightWheel;
};

}
