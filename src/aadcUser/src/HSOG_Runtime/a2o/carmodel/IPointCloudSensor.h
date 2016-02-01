#pragma once

#include "ISensor.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for a depth camera sensor.
 * 
 * \author Stefan Glaser
 */
class IPointCloudSensor : public virtual ISensor {
public:
  typedef boost::shared_ptr<IPointCloudSensor> Ptr;
  typedef boost::shared_ptr<const IPointCloudSensor> ConstPtr;

  virtual ~IPointCloudSensor(){};
  
  virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const = 0;
};

}
