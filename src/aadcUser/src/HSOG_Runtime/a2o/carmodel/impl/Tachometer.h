#pragma once

#include "Sensor.h"
#include "carmodel/ITachometer.h"
#include "meta/ISensorConfig.h"

#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The Tachometer sensor representation.
 * 
 * \author Stefan Glaser
 */
class Tachometer : public Sensor, public virtual ITachometer {
public:
  typedef boost::shared_ptr<Tachometer> Ptr;
  typedef boost::shared_ptr<const Tachometer> ConstPtr;

  Tachometer(ISensorConfig::ConstPtr config);
  virtual ~Tachometer();
  
  virtual const double& getRPM() const;
  
  virtual const bool update(IPerception::ConstPtr perception);
  
protected:
  double _rpm = 0;
  
  int _frequencyCounter;
};

}
