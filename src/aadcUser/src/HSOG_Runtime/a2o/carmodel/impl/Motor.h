#pragma once

#include "Actuator.h"
#include "carmodel/IMotor.h"
#include "meta/IActuatorConfig.h"
#include "utils/concurrency/Stopwatch.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * The Motor class represents a free rotating motor without limits.
 * This type of motor is controlled by a target speed (rpm or voltage) of the motor.
 * 
 * \author Stefan Glaser
 */
class Motor : public Actuator, public virtual IMotor {
public:
  typedef boost::shared_ptr<Motor> Ptr;
  typedef boost::shared_ptr<const Motor> ConstPtr;

  Motor(IActuatorConfig::ConstPtr config);
  virtual ~Motor();
  
  virtual const double& getSpeed() const;
  virtual void setSpeed(const double& speed);
  
  virtual const bool update(IPerception::ConstPtr perception);
  virtual const bool reflectAction(IAction::Ptr action) const;
  virtual const bool isInitialized() const;
  
protected:
  /** The current speed of the motor */
  double _speed;

  Stopwatch* _stopWatch;
};

}
