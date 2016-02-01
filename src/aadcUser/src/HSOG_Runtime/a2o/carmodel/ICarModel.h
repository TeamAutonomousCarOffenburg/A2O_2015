#pragma once

#include "ISensor.h"
#include "IDistanceSensor.h"
#include "ITachometer.h"
#include "IGyroscope.h"
#include "IAccelerometer.h"
#include "ICamera.h"
#include "IPointCloudSensor.h"
#include "IActuator.h"
#include "IServoDrive.h"
#include "IMotor.h"
#include "ILight.h"
#include "IAxle.h"
#include "IManeuverStatusActuator.h"
#include "action/IAction.h"
#include "perception/IPerception.h"
#include "utils/geometry/drive/DriveCalculator.h"
#include "maneuver/IManeuverConstants.h"


#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * General Interface for the car model component.
 * The ICarModel represents all sensors and actuators of the car,
 * as well as all information about the car.
 *
 * \author Stefan Glaser
 */
class ICarModel {
public:
  typedef boost::shared_ptr<ICarModel> Ptr;
  typedef boost::shared_ptr<const ICarModel> ConstPtr;
  
  virtual ~ICarModel(){};
  
  virtual IAxle::ConstPtr getFrontAxle() const = 0;
  virtual IAxle::ConstPtr getRearAxle() const = 0;
  virtual DriveCalculator::ConstPtr getDriveCalculator() const = 0;
  
  virtual const Eigen::Vector3d getBoundingBox() const = 0;
  
  virtual ISensor::ConstPtr getSensor(const std::string& name) const = 0;
  virtual IDistanceSensor::ConstPtr getDistanceSensor(const std::string& name) const = 0;
  virtual ITachometer::ConstPtr getTachometer(const std::string& name) const = 0;
  virtual IGyroscope::Ptr getGyro(const std::string& name) const = 0;
  virtual IAccelerometer::ConstPtr getAccelerometer(const std::string& name) const = 0;
  virtual ICamera::ConstPtr getCamera(const std::string& name) const = 0;
  virtual IPointCloudSensor::ConstPtr getPointCloudSensor(const std::string& name) const = 0;
  
  virtual IActuator::Ptr getActuator(const std::string& name) = 0;
  virtual IActuator::ConstPtr getActuator(const std::string& name) const = 0;
  virtual IServoDrive::Ptr getServoDrive(const std::string& name) = 0;
  virtual IServoDrive::ConstPtr getServoDrive(const std::string& name) const = 0;
  virtual IMotor::Ptr getMotor(const std::string& name) = 0;
  virtual IMotor::ConstPtr getMotor(const std::string& name) const = 0;
  virtual ILight::Ptr getLight(const std::string& name) = 0;
  virtual ILight::ConstPtr getLight(const std::string& name) const = 0;
  virtual IManeuverStatusActuator::Ptr getManeuverStatusActuator(const std::string& name) = 0;
  virtual IManeuverStatusActuator::ConstPtr getManeuverStatusActuator(const std::string& name) const = 0;
  
  virtual IServoDrive::Ptr getSteeringServo() = 0;
  virtual IServoDrive::ConstPtr getSteeringServo() const = 0;
  virtual IMotor::Ptr getMainMotor() = 0;
  virtual IMotor::ConstPtr getMainMotor() const = 0;
  
  /** Set the steering angle to the given angle. */
  virtual void steer(const Angle& angle) = 0;
  
  /** Steer towards the given local target point (Set the steering angle as the angle to the given point). */
  virtual void steerTo(const Eigen::Vector2d& localTarget) = 0;
  
  /** Steer towards the given local target pose by following the radius of a double arc to the target point. */
  virtual void steerDoubleArcTo(const Pose2D& localTarget) = 0;
  
  /** Steer towards the given local target point following an circular arc. */
  virtual void steerArcTo(const Eigen::Vector2d& localTarget) = 0;
  
  /** Try to steer such that the car drives the specified radius. */
  virtual void steerRadius(const double& radius) = 0;
    
   /** Called to update the car model with a new perception.
   *
   * \returns success
   */
  virtual const bool update(IPerception::ConstPtr perception) = 0;
  
   /** Called to update the Action with new actions from the actuators of the car model.
   *
   * \returns success
   */
  virtual const bool reflectActions(IAction::Ptr action) = 0;
  
  /** \brief Check if all sensors are initialized.
    * 
    * \returns true, if all sensors are properly initialized, false otherwise
    */
  virtual const bool isInitialized() const = 0;
};

}
