#pragma once

#include "IWheel.h"

#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace A2O {
  
/**
 * Interface for an axle of the car.
 * 
 * \author Stefan Glaser
 */
class IAxle {
public:
  typedef boost::shared_ptr<IAxle> Ptr;
  typedef boost::shared_ptr<const IAxle> ConstPtr;
  
  ~IAxle() {};
  
  /** Retrieve the name of this axle.
    * \returns the name of this axle
    */
  virtual const std::string& getName() const = 0;
  
  /** Retrieve the position of the axle.
    * \returns the axle position
    */
  virtual const Eigen::Vector3d& getPosition() const = 0;
  
  /** Retrieve the length of the axle (the distance from the left to the right wheel)
    * \returns the axle length
    */
  virtual const double& getLength() const = 0;
  
  /** Retrieve the left wheel mounted on this axle.
    * \returns the left wheel of this axle
    */
  virtual IWheel::ConstPtr getLeftWheel() const = 0;
  
  /** Retrieve the right wheel mounted on this axle.
    * \returns the right wheel of this axle
    */
  virtual IWheel::ConstPtr getRightWheel() const = 0;
};

}
