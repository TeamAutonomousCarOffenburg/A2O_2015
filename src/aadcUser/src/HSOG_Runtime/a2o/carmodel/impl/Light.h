#pragma once

#include "Actuator.h"
#include "carmodel/ILight.h"
#include "meta/IActuatorConfig.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * The Light class represents a light of the car (headlight, indicators, etc.).
 * 
 * \author Stefan Glaser
 */
class Light : public Actuator, public virtual ILight {
public:
  typedef boost::shared_ptr<Light> Ptr;
  typedef boost::shared_ptr<const Light> ConstPtr;

  Light(IActuatorConfig::ConstPtr config);
  virtual ~Light();
  
  virtual const bool& isOn() const;
  virtual void setState(const bool& active);
  virtual void turnOn();
  virtual void turnOff();
  
  virtual const bool update(IPerception::ConstPtr perception);
  virtual const bool reflectAction(IAction::Ptr action) const;
  
protected:
  /** The state of the light. */
  bool _active = false;
};

}
