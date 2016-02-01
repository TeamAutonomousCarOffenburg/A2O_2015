#pragma once

#include <thread>
#include <mutex>

#include "../IVisibleObject.h"
#include "../IObjectDetection.h"
#include "utils/concurrency/BlockingQueue.h"

#include "IResource.h"
#include "PointCloudWorker.h"
#include "SensorFusion.h"

#include <pcl/filters/extract_indices.h>







namespace A2O {
  
/**
 * class manages obstacle detection
 *
 * \author Peter Walden
 */

class ObjectDetection : public virtual IObjectDetection, public virtual IResource{

public:
  ObjectDetection();
   virtual ~ObjectDetection();
   ObjectDetection(const ObjectDetection&);

  
  void start();
  void stop();
  void update();
  
  virtual void setData(const Pose2D& thisCarPose, IPointCloudSensor::ConstPtr pointCloudSensor,
	  std::vector<IDistanceSensor::ConstPtr> sensorsIR, std::vector<IDistanceSensor::ConstPtr> sensorsUS);

  std::vector<IVisibleObject::Ptr> getObjectResult();
  
  virtual void setResult(std::vector<IVisibleObject::Ptr> results);
  virtual const Pose2D getPose();
  virtual const IPointCloudSensor::ConstPtr getPointCloudSensor();
  virtual const std::vector<AlignedBoundingBox3D::Ptr> getDistanceSensorData();
  virtual const std::vector<AlignedBoundingBox3D::Ptr> getPointCloudSensorData();
  virtual void setPointCloudSensorData(const std::vector<AlignedBoundingBox3D::Ptr> data);



  
private:
  void processDistanceSensorData();
  void save(IDistanceSensor::ConstPtr sensor, Pose2D& carPose);
  
  long _irLastMeasurementTime;
  long _usLastMeasurementTime;
  
  Pose2D _carPose;
  
  std::vector< IDistanceSensor::ConstPtr > _sensorsIR;
  std::vector< IDistanceSensor::ConstPtr > _sensorsUS;
  
  IPointCloudSensor::ConstPtr _pointCloudSensor;
  
  PointCloudWorker* _pcWorker;
  SensorFusion* _sensorFusion;

  std::mutex _resultLock;
  std::mutex _dataLock;
  std::mutex _pointCloudDataLock;
  std::vector<IVisibleObject::Ptr> _results;
  BlockingQueue<pcl::PointXYZ, 500> *_distanceValues;
  std::vector<AlignedBoundingBox3D::Ptr> _pointCloudData;
};
}
