#pragma once

#include "IActuator.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Interface for a motor (endless rotation).
 * 
 * \author Stefan Glaser
 */
class IMotor : public virtual IActuator {
public:
  typedef boost::shared_ptr<IMotor> Ptr;
  typedef boost::shared_ptr<const IMotor> ConstPtr;

  virtual ~IMotor(){};
  
  /** Retrieve the speed of the motor.
   *
   * \returns The position of the motor.
   */
  virtual const double& getSpeed() const = 0;
  
  /** 
   * Set the target speed of the motor.
   * 
   * \param speed - the target speed
   */
  virtual void setSpeed(const double& speed) = 0;
};

}
