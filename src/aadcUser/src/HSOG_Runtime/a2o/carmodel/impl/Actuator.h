#pragma once

#include "carmodel/IActuator.h"
#include "meta/IActuatorConfig.h"
#include "action/IAction.h"
#include "perception/IPerception.h"

#include <string>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The Actuator class provides the basic properties that each actuators have in common.
 * Each actuator has a unique name.
 * 
 * \author Stefan Glaser
 */
class Actuator : public virtual IActuator {
public:
  typedef boost::shared_ptr<Actuator> Ptr;
  typedef boost::shared_ptr<const Actuator> ConstPtr;

  Actuator(IActuatorConfig::ConstPtr config);
  Actuator(const std::string& name,
	   const std::string& perceptorName,
	   const std::string& effectorName);
  virtual ~Actuator();
  
  virtual const std::string& getName() const;
  virtual const std::string& getPerceptorName() const;
  virtual const std::string& getEffectorName() const;
  virtual const long& getLastMeasurementTime() const;
  virtual const bool isInitialized() const;
  
  virtual const bool update(IPerception::ConstPtr perception) = 0;
  virtual const bool reflectAction(IAction::Ptr action) const = 0;
  
protected:
  /** The name of the actuator. */
  std::string _name;
  
  /** The name of the corresponding perceptor. */
  std::string _perceptorName;
  
  /** The name of the corresponding effector. */
  std::string _effectorName;
  
  /** The time of the last measurement. */
  long _lastMeasurementTime;
};

}
