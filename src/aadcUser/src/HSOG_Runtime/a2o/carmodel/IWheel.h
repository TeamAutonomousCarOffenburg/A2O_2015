#pragma once

#include "utils/geometry/Pose2D.h"

#include <string>
#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace A2O {

/**
 * Interface for a wheel mounted to an axle of the car.
 * 
 * \author Stefan Glaser
 */
class IWheel {
public:
  typedef boost::shared_ptr<IWheel> Ptr;
  typedef boost::shared_ptr<const IWheel> ConstPtr;
  
  ~IWheel() {};
  
  /** Retrieve the pose of this wheel.
    * \returns the pose of this wheel
    */
  virtual const Pose2D& getPose() const = 0;
  
  /** Retrieve the diameter of the wheel.
    * \returns the wheel diameter
    */
  virtual const double& getDiameter() const = 0;
  
  /** Retrieve the name of the tachometer attached to this wheel.
    * \returns the name of the tachometer attached to this wheel
    */
  virtual const std::string& getTachometerName() const = 0;
  
  /** Check if this wheel has an attached tachometer.
    * \returns true, if this wheel has an tachometer attached, false otherwise
    */
  virtual const bool hasTachometer() const = 0;
  
  /** Retrieve the "rounds per second" this wheel spins at.
    * \returns the rounds per second
    */
  virtual const double getRPS() const = 0;
  
  /** Retrieve the speed of this wheel in meter per second.
    * The wheel speed is defined as the circular distance that a point on the wheel travels during one second.
    * \returns the distance per second of this wheel
    */
  virtual const double getSpeed() const = 0;
};

}
