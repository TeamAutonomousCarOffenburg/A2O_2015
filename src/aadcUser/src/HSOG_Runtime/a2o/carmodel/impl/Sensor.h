#pragma once

#include "carmodel/ISensor.h"
#include "perception/IPerception.h"
#include "meta/ISensorConfig.h"
#include "utils/geometry/Pose3D.h"

#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/StdVector>
#include <boost/smart_ptr.hpp>

// In order to be able to use vector2d
// -in stl containers, this line is mandatory.
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector2d)


namespace A2O {
  
/**
 * The Sensor class provides the basic properties that each sensors have in common.
 * Each sensor has a unique name and mounting location on the car.
 * 
 * \author Stefan Glaser
 */
class Sensor : public virtual ISensor {
public:
  typedef boost::shared_ptr<Sensor> Ptr;
  typedef boost::shared_ptr<const Sensor> ConstPtr;

  Sensor(ISensorConfig::ConstPtr config);
  Sensor(const std::string& name,
	 const std::string& perceptorName,
	 const Pose3D& pose);
  virtual ~Sensor();
  
  virtual const std::string& getName() const;
  virtual const std::string& getPerceptorName() const;
  virtual const Pose3D& getPose() const;
  virtual const long& getLastMeasurementTime() const;
  
  virtual const Eigen::Vector3d toCarSystem(const Eigen::Vector3d& localPos) const;
  virtual const Eigen::Quaterniond toCarSystem(const Eigen::Quaterniond& localOrientation) const;
  virtual const Pose3D toCarSystem(const Pose3D& localPose) const;
  virtual const bool isInitialized() const;
  
  /** Called to update the sensor with a new perception.
   *
   * \returns success
   */
  virtual const bool update(IPerception::ConstPtr perception) = 0;
  
protected:
  /** The name of the sensor. */
  std::string _name;
  
  /** The name of the perceptor. */
  std::string _perceptorName;
  
  /** The time of the last measurement. */
  long _lastMeasurementTime;
  
  /** The position of the sensor. */
  Pose3D _pose;
};

}
