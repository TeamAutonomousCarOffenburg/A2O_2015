#pragma once

#include <string>
#include <map>
#include <boost/smart_ptr.hpp>
#include "IManeuverConstants.h"

namespace A2O {

/**
 * Basic Interface for all maneuvers of the car.
 * A maneuver represents some kind of control strategy to drive the car.
 * 
 * \author Stefan Glaser
 */
class IManeuver {
public:
  typedef boost::shared_ptr<IManeuver> Ptr;
  typedef boost::shared_ptr<const IManeuver> ConstPtr;
  
  virtual ~IManeuver(){};
  
  /** Retrieve the name of the maneuver.
   * 
   * \returns the name of the maneuver
   */
  virtual const std::string& getName() const = 0;
  
  /** Called to continously perform this maneuver. */
  virtual void perform() = 0;

  /** Returns if the maneuver failed / is okay */
  virtual const ManeuverStatus getStatus() const = 0;

};

typedef boost::shared_ptr<std::map<std::string, IManeuver::Ptr>> IManeuverMapPtr;
typedef boost::shared_ptr<const std::map<std::string, IManeuver::ConstPtr>> IManeuverMapConstPtr;

}
