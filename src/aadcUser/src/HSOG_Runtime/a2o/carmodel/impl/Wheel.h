#pragma once

#include "carmodel/IWheel.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The Wheel class represents an wheel mounted to an axle of the car.
 * 
 * \author Stefan Glaser
 */
class Wheel : public virtual IWheel {
public:
  typedef boost::shared_ptr<Wheel> Ptr;
  typedef boost::shared_ptr<const Wheel> ConstPtr;

  Wheel(const double& diameter,
	const Pose2D& pose,
	std::string tachoName = "");
  virtual ~Wheel();
  
  virtual const double& getDiameter() const;
  virtual const Pose2D& getPose() const;
  virtual const std::string& getTachometerName() const;
  virtual const bool hasTachometer() const;
  virtual const double getRPS() const;
  virtual const double getSpeed() const;
  
protected:
  const double _diameter;
  
  const Pose2D _pose;
  
  std::string _tachoName;
};

}
