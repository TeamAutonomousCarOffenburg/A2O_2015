#pragma once

#include "Sensor.h"
#include "meta/ICameraConfig.h"
#include "carmodel/IPointCloudSensor.h"

#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The DepthCamera class represents an arbitrary RGB-D camera.
 * 
 * \author Stefan Glaser
 */
class PointCloudSensor : public Sensor, public virtual IPointCloudSensor {
public:
  typedef boost::shared_ptr<PointCloudSensor> Ptr;
  typedef boost::shared_ptr<const PointCloudSensor> ConstPtr;

  PointCloudSensor(ICameraConfig::ConstPtr config);
  virtual ~PointCloudSensor();
  
  virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;
  
  virtual const bool update(IPerception::ConstPtr perception);
  
protected:
  /** The point cloud. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
};

}
