#pragma once

#include "utils/geometry/Pose3D.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for all known sensors of the system.
 * Each sensor specifies a unique name and mounting location on the car.
 * 
 * \author Stefan Glaser
 */
class ISensor {
public:
  typedef boost::shared_ptr<ISensor> Ptr;
  typedef boost::shared_ptr<const ISensor> ConstPtr;

  virtual ~ISensor(){};
  
  /** Retrieve the name of the sensor.
   *
   * \returns The sensor name.
   */
  virtual const std::string& getName() const = 0;
  
  /** Retrieve the perceptor name corresponding to this sensor.
   *
   * \returns The corresponding perceptor name.
   */
  virtual const std::string& getPerceptorName() const = 0;
  
  /** Retrieve the pose of the sensor relative to the center of the car.
   * 
   * \returns The sensor pose.
   */
  virtual const Pose3D& getPose() const = 0;
  
  /** Retrieve the time this sensor received the last update.
   * 
   * \returns The time of the last sensor update.
   */
  virtual const long& getLastMeasurementTime() const = 0;
  
  /** \brief Transform the given local position into the car koordinate system.
   *
   * \returns A vector to the local position in the car system.
   */
  virtual const Eigen::Vector3d toCarSystem(const Eigen::Vector3d& localPos) const = 0;
  
  /** \brief Transform the given local orientation into the car koordinate system.
   *
   * \returns A quaternion, representing the local orientation in the car system.
   */
  virtual const Eigen::Quaterniond toCarSystem(const Eigen::Quaterniond& localOrientation) const = 0;
  
  /** \brief Transform the given local pose into the car koordinate system.
   *
   * \returns A pose, representing the local pose in the car system.
   */
  virtual const Pose3D toCarSystem(const Pose3D& localPose) const = 0;
  
  /** \brief Check if this sensor is properly initialized or not.
    *
    * \returns true, if this sensor is properly initialized, false otherwise
    */
  virtual const bool isInitialized() const = 0;
};

}
