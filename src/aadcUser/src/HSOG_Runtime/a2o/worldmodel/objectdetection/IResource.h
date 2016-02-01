#pragma once

#include <vector>
#include "../IVisibleObject.h"
#include "carmodel/IPointCloudSensor.h"
#include "utils/geometry/AlignedBoundingBox3D.h"

namespace A2O {
  
  /**
   * Interface: avoidance of cyclic dependencies
   * 
   */

class IResource {
public:

  virtual void setResult(std::vector<IVisibleObject::Ptr> results) = 0;
  virtual const Pose2D getPose() = 0;
  virtual const IPointCloudSensor::ConstPtr getPointCloudSensor() = 0;
  virtual const std::vector<AlignedBoundingBox3D::Ptr> getDistanceSensorData() = 0;
  virtual const std::vector<AlignedBoundingBox3D::Ptr> getPointCloudSensorData() = 0;
  virtual void setPointCloudSensorData(const std::vector<AlignedBoundingBox3D::Ptr> data) = 0;
};
}
  