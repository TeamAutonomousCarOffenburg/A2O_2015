#include "ObjectDetection.h"

#define DISTANCE_VALUES_BUFFER_SIZE 500

using namespace A2O;

ObjectDetection::ObjectDetection()
{
  _irLastMeasurementTime = 0;
  _usLastMeasurementTime = 0;
  _pcWorker = new PointCloudWorker(this);
  _sensorFusion = new SensorFusion(this);
  _distanceValues = new BlockingQueue<pcl::PointXYZ, DISTANCE_VALUES_BUFFER_SIZE>();
}

ObjectDetection::ObjectDetection(const ObjectDetection&)
{
}

ObjectDetection::~ObjectDetection()
{       
  free(_pcWorker);
  free(_sensorFusion);
  free(_distanceValues);
}


void ObjectDetection::start()
{
  _pcWorker->start("PointCloudWorker");
  _sensorFusion->start("SensorFusion");
}

void ObjectDetection::stop()
{
  _pcWorker->stop();
  _sensorFusion->stop();
  _distanceValues->clear();
  std::cout << "ObjectDetection: bye bye!" << std::endl;
}

void ObjectDetection::update()
{
  processDistanceSensorData();
}

std::vector<IVisibleObject::Ptr> ObjectDetection::getObjectResult()
{
  std::lock_guard<std::mutex> guard(_resultLock);
  std::vector<IVisibleObject::Ptr> tmpResult = _results;
  _results.clear();
  return tmpResult;
}

void ObjectDetection::setResult(std::vector<IVisibleObject::Ptr> results)
{
  std::lock_guard<std::mutex> guard(_resultLock);
  _results = results;
}

void ObjectDetection::setData(const Pose2D& thisCarPose, IPointCloudSensor::ConstPtr pointCloudSensor, 
			      std::vector< IDistanceSensor::ConstPtr > sensorsIR, std::vector< IDistanceSensor::ConstPtr > sensorsUS)
{
  std::lock_guard<std::mutex> guard(_dataLock);
  _carPose = thisCarPose;
  _pointCloudSensor = pointCloudSensor;
  _sensorsIR = sensorsIR;
  _sensorsUS = sensorsUS;
}

const Pose2D ObjectDetection::getPose()
{
  std::lock_guard<std::mutex> guard(_dataLock);
  return _carPose;
}


const IPointCloudSensor::ConstPtr ObjectDetection::getPointCloudSensor()
{
  std::lock_guard<std::mutex> guard(_dataLock);
  return _pointCloudSensor;
}

const std::vector< AlignedBoundingBox3D::Ptr > ObjectDetection::getDistanceSensorData()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  int size = _distanceValues->size();
  for(int i = 0; i < size; i++)
  {
    cloud->push_back(_distanceValues->at(i));
  }
  std::vector <pcl::PointIndices> clusters = PCLUtils::segmentation(cloud, 4, 5);
  std::vector<AlignedBoundingBox3D::Ptr> result;

  for(pcl::PointIndices cluster : clusters)
  {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractedCloudPtr (new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud (cloud);
    pcl::PointIndices::Ptr indicesPtr =  pcl::PointIndices::Ptr(new pcl::PointIndices(cluster));
    extract.setIndices (indicesPtr);
    extract.setNegative (false);
    extract.filter (*extractedCloudPtr);
    AlignedBoundingBox3D::Ptr box = PCLUtils::alignedBoundingBox(extractedCloudPtr);
    result.push_back(box);
  }
  return result; 
}


const std::vector< AlignedBoundingBox3D::Ptr > ObjectDetection::getPointCloudSensorData()
{
  std::lock_guard<std::mutex> guard(_pointCloudDataLock);
  return _pointCloudData;
}


void ObjectDetection::setPointCloudSensorData(const std::vector< AlignedBoundingBox3D::Ptr > data)
{
  std::lock_guard<std::mutex> guard(_pointCloudDataLock);
  _pointCloudData = data;
}


void ObjectDetection::processDistanceSensorData()
{
  long measurementTime;
  Pose2D carPose = getPose();

  if(_sensorsIR.size() > 0)
  {
    measurementTime = _sensorsIR.at(0)->getLastMeasurementTime();
    if(measurementTime > _irLastMeasurementTime)
    {
      std::vector< IDistanceSensor::ConstPtr > sensorsIR;
      {
	std::lock_guard<std::mutex> guard(_dataLock);
	sensorsIR = _sensorsIR;
      }
      
      _irLastMeasurementTime = measurementTime;      
      for(IDistanceSensor::ConstPtr sensor  : sensorsIR)
      {
	if(sensor->inRange())
	{
	  save(sensor, carPose);
	}
      }
    }
  } 
  
  if(_sensorsUS.size() > 0)
  {
    measurementTime = _sensorsUS.at(0)->getLastMeasurementTime();
    if(measurementTime > _usLastMeasurementTime)
    {
      std::vector< IDistanceSensor::ConstPtr > sensorsUS;
      {
	std::lock_guard<std::mutex> guard(_dataLock);
	sensorsUS = _sensorsUS;
      }
      _usLastMeasurementTime = measurementTime;
  //     std::cout << "DistanceSensorWorker: US UPDATE" << std::endl;

      for(IDistanceSensor::ConstPtr sensor  : sensorsUS)
      {
	if(sensor->inRange())
	{
	  save(sensor, carPose);
	}
      }
    }
  }
}

void ObjectDetection::save(IDistanceSensor::ConstPtr sensor, Pose2D& carPose)
{
  double sensorValue = sensor->getDistance();
  Eigen::Vector3d value = Eigen::Vector3d(sensorValue, 0, 0);
  Eigen::Quaterniond orientation = sensor->getPose().getOrientation();	
  Eigen::Vector3d position = sensor->getPose().getPosition();
  Eigen::Vector2d position2D(position(0), position(1));
  position2D = carPose * position2D;
  position = Eigen::Vector3d(position2D(0), position2D(1), position(2));
  Eigen::Vector3d location = orientation._transformVector(value) + position;
  // USS: add 3 points
  if(sensor->getScanWidth() > 0.05)
  {
    Angle angle = Angle::to(location(0), location(1));
    angle = angle.normal();
    Eigen::Vector2d offset2d1 = angle.getVector(sensor->getScanWidth() / 2);
    Eigen::Vector3d location2 = location + Eigen::Vector3d(offset2d1(0), offset2d1(1), 0);
    pcl::PointXYZ point2(location2(0), location2(1), location2(2));
    _distanceValues->push(point2);

    angle = angle.opposite();
    Eigen::Vector2d offset2d2 = angle.getVector(sensor->getScanWidth() / 2);
    Eigen::Vector3d location3 = location + Eigen::Vector3d(offset2d2(0), offset2d2(1), 0);
    pcl::PointXYZ point3(location3(0), location3(1), location3(2));
    _distanceValues->push(point3);
  }
  pcl::PointXYZ point(location(0), location(1), location(2));
  _distanceValues->push(point);
}






