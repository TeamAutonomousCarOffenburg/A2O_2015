#pragma once

#include "IActuator.h"

#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for a light (headlight, indicator, backlight, breaking-light, etc.).
 * 
 * \author Stefan Glaser
 */
class ILight : public virtual IActuator {
public:
  typedef boost::shared_ptr<ILight> Ptr;
  typedef boost::shared_ptr<const ILight> ConstPtr;

  virtual ~ILight(){};
  
  /** Retrieve the state of the light (on or off).
   *
   * \returns true if the light is on, false otherwise
   */
  virtual const bool& isOn() const = 0;
  
  /** 
   * Set the state of the light.
   * 
   * \param active - true to activate the light, false to deacivate
   */
  virtual void setState(const bool& active) = 0;
  
  /** Turn light on. */
  virtual void turnOn() = 0;
  
  /** Turn light off. */
  virtual void turnOff() = 0;
};

}
