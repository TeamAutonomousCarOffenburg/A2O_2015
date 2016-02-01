#include "PointCloudSensor.h"

#include "perception/IPointCloudPerceptor.h"


using namespace A2O;

PointCloudSensor::PointCloudSensor(ICameraConfig::ConstPtr config)
      : Sensor(config)
{

}

PointCloudSensor::~PointCloudSensor()
{
  
}

pcl::PointCloud< pcl::PointXYZ >::Ptr PointCloudSensor::getPointCloud() const
{
  return _cloud;
}

const bool PointCloudSensor::update(IPerception::ConstPtr perception)
{
  IPointCloudPerceptor::ConstPtr pclPerceptor = perception->getPointCloudPerceptor(_perceptorName);
  
  if (pclPerceptor.get()) {
    _cloud = pclPerceptor->getPointCloud();
    _lastMeasurementTime = pclPerceptor->getTime();
    return true;
  }
  
  return false;
}
