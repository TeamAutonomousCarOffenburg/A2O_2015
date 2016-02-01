#pragma once

#include "ISensor.h"

#include <boost/smart_ptr.hpp>

namespace A2O {

/**
 * Interface for a Tachometer sensor.
 * 
 * \author Stefan Glaser
 */
class ITachometer : public virtual ISensor {
public:
  typedef boost::shared_ptr<ITachometer> Ptr;
  typedef boost::shared_ptr<const ITachometer> ConstPtr;

  virtual ~ITachometer(){};
  
  /** Retrieve the most recent measured rounds per minute.
   * 
   * \returns the rounds per minute
   */
  virtual const double& getRPM() const = 0;
};

}
