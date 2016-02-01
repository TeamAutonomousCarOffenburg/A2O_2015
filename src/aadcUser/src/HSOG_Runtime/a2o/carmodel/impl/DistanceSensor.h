#pragma once

#include "Sensor.h"
#include "carmodel/IDistanceSensor.h"
#include "meta/IDistanceSensorConfig.h"

#include <boost/smart_ptr.hpp>
#include <mutex>
#include <condition_variable>
#include <atomic>



namespace A2O {
  
/**
 * The DistanceSensor (InfraRed, UltraSound, etc.) representation.
 * 
 * \author Stefan Glaser
 */
class DistanceSensor : public Sensor, public virtual IDistanceSensor {
public:
  typedef boost::shared_ptr<DistanceSensor> Ptr;
  typedef boost::shared_ptr<const DistanceSensor> ConstPtr;

  DistanceSensor(IDistanceSensorConfig::ConstPtr config);
  virtual ~DistanceSensor();
  
  virtual const double& getMinDistance() const;
  virtual const double& getMaxDistance() const;
  virtual const double& getScanWidth() const;
  virtual const double& getDistance() const;
  virtual const double& getLimitedDistance() const;
  virtual const bool inRange() const;
  virtual const double getPercent() const;
  
  virtual const bool update(IPerception::ConstPtr perception);
  
protected:
  /** The minimum distance this sensor is able to measure properly. */
  double _minDistance;
  
  /** The maximum distance this sensor is able to measure properly. */
  double _maxDistance;
  
  /** The scan with of this distance sensor. */
  double _scanWidth;
  
  /** The currently measured distance. */
  double _distance;
  
  mutable std::mutex _mutex;
};

}
