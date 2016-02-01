#include "CarModel.h"

#include "Gyroscope.h"
#include "Accelerometer.h"
#include "IMU.h"
#include "DistanceSensor.h"
#include "Tachometer.h"
#include "Camera.h"
#include "PointCloudSensor.h"

#include "ServoDrive.h"
#include "Motor.h"
#include "Light.h"

#include "ManeuverStatusActuator.h"
#include <utils/geometry/Geometry.h>
#include "AADCCar.h"


using namespace A2O;
using namespace Eigen;

CarModel::CarModel(ICarMetaModel::ConstPtr carMetaModel)
{
  // Create axles
  _frontAxle = boost::make_shared<Axle>(carMetaModel->getFrontAxle());
  _rearAxle = boost::make_shared<Axle>(carMetaModel->getRearAxle());
  
  // Create drive calculator
  double axleDistance = _frontAxle->getPosition()(0) - _rearAxle->getPosition()(0);
  _driveCalculator = boost::make_shared<DriveCalculator>(_rearAxle->getLength(), axleDistance);
  
  // Add sensors
  // Add IMU sensors
  std::vector<ISensorConfig::ConstPtr> sensorConfigs = carMetaModel->getIMUConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    IMU::Ptr imuSensor(boost::make_shared<IMU>(sensorConfigs[i]));
    _sensors.insert(make_pair(imuSensor->getName(), imuSensor));
  }

  // Add Gyroscope sensors
  sensorConfigs = carMetaModel->getGyroConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    Gyroscope::Ptr sensor(boost::make_shared<Gyroscope>(sensorConfigs[i]));
    _sensors.insert(make_pair(sensor->getName(), sensor));
  }
  
  // Add Accelerometer sensors
  sensorConfigs = carMetaModel->getAccelerometerConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    Accelerometer::Ptr sensor(boost::make_shared<Accelerometer>(sensorConfigs[i]));
    _sensors.insert(make_pair(sensor->getName(), sensor));
  }
  
  // Add InfraRed sensors
  std::vector<IDistanceSensorConfig::ConstPtr> dsConfigs = carMetaModel->getInfraRedConfigs();
  for (unsigned int i = 0; i < dsConfigs.size(); i++) {
    DistanceSensor::Ptr sensor(boost::make_shared<DistanceSensor>(dsConfigs[i]));
    _sensors.insert(make_pair(sensor->getName(), sensor));
  }
  
  // Add UltraSound sensors
  dsConfigs = carMetaModel->getUltraSoundConfigs();
  for (unsigned int i = 0; i < dsConfigs.size(); i++) {
    DistanceSensor::Ptr sensor(boost::make_shared<DistanceSensor>(dsConfigs[i]));
    _sensors.insert(make_pair(sensor->getName(), sensor));
  }
  
  // Add Rotation sensors
  sensorConfigs = carMetaModel->getRotationConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    Tachometer::Ptr sensor(boost::make_shared<Tachometer>(sensorConfigs[i]));
    _sensors.insert(make_pair(sensor->getName(), sensor));
  }
  
  // Add Voltage sensors
  // TODO
  
  // Add Camera sensros
  std::vector<ICameraConfig::ConstPtr> cameraConfigs = carMetaModel->getCameraConfigs();
  for (unsigned int i = 0; i < cameraConfigs.size(); i++) {
    Camera::Ptr sensor(boost::make_shared<Camera>(cameraConfigs[i]));
    _sensors.insert(make_pair(sensor->getName(), sensor));
  }
  
  // Add DepthCamera sensors
  cameraConfigs = carMetaModel->getDepthCameraConfigs();
  for (unsigned int i = 0; i < cameraConfigs.size(); i++) {
    PointCloudSensor::Ptr sensor(boost::make_shared<PointCloudSensor>(cameraConfigs[i]));
    _sensors.insert(make_pair(sensor->getName(), sensor));
  }
  
  // Add Actuators
  // Add ServoDrive actuators
  std::vector<IServoDriveConfig::ConstPtr> servoDriveConfigs = carMetaModel->getServoDriveConfigs();
  for (unsigned int i = 0; i < servoDriveConfigs.size(); i++) {
    ServoDrive::Ptr actuator(boost::make_shared<ServoDrive>(servoDriveConfigs[i]));
    _actuators.insert(make_pair(actuator->getName(), actuator));
  }
  
  // Add Motor actuators
  std::vector<IActuatorConfig::ConstPtr> actuatorConfigs = carMetaModel->getMotorConfigs();
  for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
    Motor::Ptr actuator(boost::make_shared<Motor>(actuatorConfigs[i]));
    _actuators.insert(make_pair(actuator->getName(), actuator));
  }
  
  // Add Light actuators
  actuatorConfigs = carMetaModel->getLightConfigs();
  for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
    Light::Ptr actuator(boost::make_shared<Light>(actuatorConfigs[i]));
    _actuators.insert(make_pair(actuator->getName(), actuator));
  }

  actuatorConfigs = carMetaModel->getManeuverStatusConfigs();
  for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
	  ManeuverStatusActuator::Ptr actuator(boost::make_shared<ManeuverStatusActuator>(actuatorConfigs[i]));
	  _actuators.insert(make_pair(actuator->getName(), actuator));
  }
  
  // Buffer steering servo and main motor
  _steeringServo = getServoDrive(carMetaModel->getSteeringServoName());
  _mainMotor = getMotor(carMetaModel->getMainMotorName());
}

CarModel::~CarModel()
{
}

IServoDrive::Ptr CarModel::getSteeringServo()
{
  return _steeringServo;
}

IServoDrive::ConstPtr CarModel::getSteeringServo() const
{
  return _steeringServo;
}

IMotor::Ptr CarModel::getMainMotor()
{
  return _mainMotor;
}

IMotor::ConstPtr CarModel::getMainMotor() const
{
  return _mainMotor;
}

IAxle::ConstPtr CarModel::getFrontAxle() const
{
  return boost::dynamic_pointer_cast<const IAxle>(_frontAxle);
}

IAxle::ConstPtr CarModel::getRearAxle() const
{
  return boost::dynamic_pointer_cast<const IAxle>(_rearAxle);
}

DriveCalculator::ConstPtr CarModel::getDriveCalculator() const
{
  return _driveCalculator;
}

const Eigen::Vector3d CarModel::getBoundingBox() const
{
  return _boundingBox;
}

ISensor::ConstPtr CarModel::getSensor(const std::string& name) const
{
  return _sensors.at(name);
}

IDistanceSensor::ConstPtr CarModel::getDistanceSensor(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const IDistanceSensor>(_sensors.at(name));
}

ITachometer::ConstPtr CarModel::getTachometer(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const ITachometer>(_sensors.at(name));
}

IGyroscope::Ptr CarModel::getGyro(const std::string& name) const
{
  return boost::dynamic_pointer_cast<IGyroscope>(_sensors.at(name));
}

IAccelerometer::ConstPtr CarModel::getAccelerometer(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const IAccelerometer>(_sensors.at(name));
}

ICamera::ConstPtr CarModel::getCamera(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const ICamera>(_sensors.at(name));
}

IPointCloudSensor::ConstPtr CarModel::getPointCloudSensor(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const IPointCloudSensor>(_sensors.at(name));
}

IActuator::Ptr CarModel::getActuator(const std::string& name)
{
  return _actuators.at(name);;
}

IActuator::ConstPtr CarModel::getActuator(const std::string& name) const
{
  return _actuators.at(name);
}

IServoDrive::Ptr CarModel::getServoDrive(const std::string& name)
{
  return boost::dynamic_pointer_cast<IServoDrive>(_actuators.at(name));
}

IServoDrive::ConstPtr CarModel::getServoDrive(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const IServoDrive>(_actuators.at(name));
}

IMotor::Ptr CarModel::getMotor(const std::string& name)
{
  return boost::dynamic_pointer_cast<IMotor>(_actuators.at(name));
}

IMotor::ConstPtr CarModel::getMotor(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const IMotor>(_actuators.at(name));
}

ILight::Ptr CarModel::getLight(const std::string& name)
{
  return boost::dynamic_pointer_cast<ILight>(_actuators.at(name));
}

ILight::ConstPtr CarModel::getLight(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const ILight>(_actuators.at(name));
}

IManeuverStatusActuator::Ptr CarModel::getManeuverStatusActuator(const std::string& name)
{
  return boost::dynamic_pointer_cast<IManeuverStatusActuator>(_actuators.at(name));
}

IManeuverStatusActuator::ConstPtr CarModel::getManeuverStatusActuator(const std::string& name) const
{
  return boost::dynamic_pointer_cast<const IManeuverStatusActuator>(_actuators.at(name));
}

void CarModel::steer(const Angle& angle)
{
  _steeringServo->setPosition(angle.deg());
}

void CarModel::steerTo(const Vector2d& localTarget)
{
  _steeringServo->setPosition(Angle::to(localTarget).deg());
}

void CarModel::steerRadius(const double& radius)
{
  steer(_driveCalculator->calculateSteeringAngle(radius));
}

void CarModel::steerDoubleArcTo(const Pose2D& localTarget)
{
  std::vector<LineSegment> segments;
  Geometry::constructDoubleArc(Pose2D(), localTarget, segments);
  if (segments.size() > 0) {
    // Drive the second segment, if the first segment is shorster then 5cm and shorter than the second segment minus 5cm.
    if (segments.size() > 1
        && segments[0].getLength() < 0.05
        && segments[0].getLength() < segments[1].getLength() - 0.05) {
      steerRadius(segments[1].getRadius());
    } else {
      steerRadius(segments[0].getRadius());
    }
  } else {
    steer(Angle::Zero());
  }
}

void CarModel::steerArcTo(const Vector2d& localTarget)
{
  if (std::abs(localTarget(1)) < 0.0001) {
    steer(Angle::Zero());
  }
  
  double radius = (localTarget(1) + ((localTarget(0) * localTarget(0)) / localTarget(1))) / 2;
  steerRadius(radius);
}

const bool CarModel::update(IPerception::ConstPtr perception)
{
  // Update all sensors
  for (auto it = _sensors.begin(); it != _sensors.end(); it++) {
    it->second->update(perception);
  }
  
  // Update all actuators
  for (auto it = _actuators.begin(); it != _actuators.end(); it++) {
    it->second->update(perception);
  }
  
  return true;
}

const bool CarModel::reflectActions(IAction::Ptr action)
{
  for (auto it = _actuators.begin(); it != _actuators.end(); it++) {
    it->second->reflectAction(action);
  }
  
  return true;
}

const bool CarModel::isInitialized() const
{
  for (auto sensor : _sensors) {
    if (!sensor.second->isInitialized()) {
      return false;
    }
  }

  for (auto actuator : _actuators) {
    if (!actuator.second->isInitialized()) {
      return false;
    }
  }

  return true;
}
