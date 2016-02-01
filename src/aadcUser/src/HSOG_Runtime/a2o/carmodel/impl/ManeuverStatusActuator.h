#pragma once

#include "Actuator.h"
#include "carmodel/IManeuverStatusActuator.h"
#include "meta/IActuatorConfig.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

class ManeuverStatusActuator : public Actuator, public virtual IManeuverStatusActuator {
public:
  typedef boost::shared_ptr<ManeuverStatusActuator> Ptr;
  typedef boost::shared_ptr<const ManeuverStatusActuator> ConstPtr;

  ManeuverStatusActuator(IActuatorConfig::ConstPtr config);
  virtual ~ManeuverStatusActuator();
  
  virtual const ManeuverStatus& getStatus() const;
  virtual void setStatus(const ManeuverStatus& status);
  
  virtual const int& getCurrentManeuver() const;
  virtual void setCurrentManeuver(const int& maneuverId);
  
  virtual const bool update(IPerception::ConstPtr perception);
  virtual const bool reflectAction(IAction::Ptr action) const;
  
protected:
  /** The state of the ManeuverStatusActuator. */
  ManeuverStatus _status = ManeuverStatus::STARTUP;
  int _currentManeuver = 0;
};

}
