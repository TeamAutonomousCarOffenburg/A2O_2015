#pragma once
#include "carmodel/ICarModel.h"
#include "Sensor.h"
#include "Actuator.h"
#include "action/IAction.h"
#include "perception/IPerception.h"
#include "meta/ICarMetaModel.h"
#include "Axle.h"
#include "utils/geometry/drive/DriveCalculator.h"

#include <string>
#include <map>
#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * The CarModel represents all properties, measurements and actions of the car.
 * 
 * \author Stefan Glaser
 */
class CarModel : public virtual ICarModel {
public:
  typedef boost::shared_ptr<CarModel> Ptr;
  typedef boost::shared_ptr<const CarModel> ConstPtr;

  CarModel(ICarMetaModel::ConstPtr carMetaModel);
  ~CarModel();
  
  virtual IAxle::ConstPtr getFrontAxle() const;
  virtual IAxle::ConstPtr getRearAxle() const;
  virtual DriveCalculator::ConstPtr getDriveCalculator() const;
  
  virtual const Eigen::Vector3d getBoundingBox() const;
  
  virtual ISensor::ConstPtr getSensor(const std::string& name) const;
  
  virtual IDistanceSensor::ConstPtr getDistanceSensor(const std::string& name) const;
  
  virtual ITachometer::ConstPtr getTachometer(const std::string& name) const;
  
  virtual IGyroscope::Ptr getGyro(const std::string& name) const;
  
  virtual IAccelerometer::ConstPtr getAccelerometer(const std::string& name) const;
  
  virtual ICamera::ConstPtr getCamera(const std::string& name) const;
  
  virtual IPointCloudSensor::ConstPtr getPointCloudSensor(const std::string& name) const;
  
  virtual IActuator::Ptr getActuator(const std::string& name);
  virtual IActuator::ConstPtr getActuator(const std::string& name) const;
  
  virtual IServoDrive::Ptr getServoDrive(const std::string& name);
  virtual IServoDrive::ConstPtr getServoDrive(const std::string& name) const;
  
  virtual IMotor::Ptr getMotor(const std::string& name);
  virtual IMotor::ConstPtr getMotor(const std::string& name) const;
  
  virtual ILight::Ptr getLight(const std::string& name);
  virtual ILight::ConstPtr getLight(const std::string& name) const;
  
  virtual IServoDrive::Ptr getSteeringServo();
  virtual IServoDrive::ConstPtr getSteeringServo() const;
  
  virtual IMotor::Ptr getMainMotor();
  virtual IMotor::ConstPtr getMainMotor() const;

  virtual IManeuverStatusActuator::Ptr getManeuverStatusActuator(const std::string& name);
  virtual IManeuverStatusActuator::ConstPtr getManeuverStatusActuator(const std::string& name) const;
  
  virtual void steer(const Angle& angle);
  virtual void steerTo(const Eigen::Vector2d& localTarget);
  virtual void steerDoubleArcTo(const Pose2D& localTarget);
  virtual void steerArcTo(const Eigen::Vector2d& localTarget);
  virtual void steerRadius(const double& radius);
  
  virtual const bool update(IPerception::ConstPtr perception);
  virtual const bool reflectActions(IAction::Ptr action);
  virtual const bool isInitialized() const;
  
private:
  /** The steering servo. */
  IServoDrive::Ptr _steeringServo;
  
  /** The main motor of the car. */
  IMotor::Ptr _mainMotor;
  
  /** The front axle definition. */
  Axle::Ptr _frontAxle;
  
  /** The rear axle definition. */
  Axle::Ptr _rearAxle;
  
  /** The calculation module for the drive geometry of this car. */
  DriveCalculator::ConstPtr _driveCalculator;
  
  /** The map of sensors. */
  std::map<std::string, Sensor::Ptr> _sensors;
  
  /** The map of actuators. */
  std::map<std::string, Actuator::Ptr> _actuators;
  
  /** The bounding box (size) of the car. */
  Eigen::Vector3d _boundingBox;
};

}
