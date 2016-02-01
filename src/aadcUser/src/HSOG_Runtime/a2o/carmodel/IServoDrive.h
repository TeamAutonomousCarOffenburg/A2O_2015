#pragma once

#include "IActuator.h"

#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for a servo drive actuator.
 * 
 * \author Stefan Glaser
 */
class IServoDrive : public virtual IActuator {
public:
  typedef boost::shared_ptr<IServoDrive> Ptr;
  typedef boost::shared_ptr<const IServoDrive> ConstPtr;

  virtual ~IServoDrive(){};
  
  /** Retrieve the position (angle) of the servo drive.
   *
   * \returns The position of the servo drive.
   */
  virtual const double& getPosition() const = 0;
  
  /** 
   * Set the target position of the servo drive.
   * 
   * \param position - the target position
   */
  virtual void setPosition(const double& position) = 0;
  
  /** Retrieve the minimum possible position of this servo.
   * 
   * \returns The minimum possible servo position
   */
  virtual const double& getMinPosition() const = 0;
  
  /** Retrieve the maximum possible position of this servo.
   * 
   * \returns The maximum possible servo position
   */
  virtual const double& getMaxPosition() const = 0;
};

}
