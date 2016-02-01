#pragma once

#include "Actuator.h"
#include "carmodel/IServoDrive.h"
#include "meta/IServoDriveConfig.h"

#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The ServoDrive class represenst a servo drive holding a specific position.
 * 
 * \author Stefan Glaser
 */
class ServoDrive : public Actuator, public virtual IServoDrive {
public:
  typedef boost::shared_ptr<ServoDrive> Ptr;
  typedef boost::shared_ptr<const ServoDrive> ConstPtr;

  ServoDrive(IServoDriveConfig::ConstPtr config);
  virtual ~ServoDrive();
  
  virtual const double& getPosition() const;
  virtual const double& getPreviousPosition() const;
  virtual const double& getTargetPosition() const;
  virtual void setPosition(const double& position);
  virtual const double& getMinPosition() const;
  virtual const double& getMaxPosition() const;
  
  virtual const bool update(IPerception::ConstPtr perception);
  virtual const bool reflectAction(IAction::Ptr action) const;
  
protected:
  double _position;
  
  double _previousPosition;
  
  double _targetPosition;
  
  double _minPosition;
  
  double _maxPosition;
};

}
