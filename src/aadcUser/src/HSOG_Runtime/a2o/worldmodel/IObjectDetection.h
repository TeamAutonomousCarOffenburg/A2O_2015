#pragma once

#include <vector>
#include <boost/smart_ptr.hpp>
#include "IVisibleObject.h"
#include "carmodel/IPointCloudSensor.h"
#include "carmodel/IDistanceSensor.h"


namespace A2O {

// class IWorldModel;
  
/**
 * Interface for the object detection.
 *
 */
class IObjectDetection {
public:
  virtual ~IObjectDetection(){};
  virtual void stop() = 0; 
  virtual void start() = 0;
  virtual void update() = 0;
  
  virtual void setData(const Pose2D& thisCarPose, IPointCloudSensor::ConstPtr pointCloudSensor,
	  std::vector<IDistanceSensor::ConstPtr> sensorsIR, std::vector<IDistanceSensor::ConstPtr> sensorsUS) = 0;
  /** Retrieve the most recent result from object detection.
   * \return vector of objects once, on second call empty vector till new data is available.
   */
  virtual std::vector<IVisibleObject::Ptr> getObjectResult() = 0;
  
  typedef boost::shared_ptr<IObjectDetection> Ptr;
  
};
}
