#pragma once

#include "Car.h"
#include "worldmodel/IThisCar.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Representation for this (our own) car.
 *
 * \author Stefan Glaser
 */
class ThisCar : public Car, public virtual IThisCar {
public:
  typedef boost::shared_ptr<ThisCar> Ptr;
  typedef boost::shared_ptr<const ThisCar> ConstPtr;
  
  ThisCar(const std::string& name,
	  const Pose2D& pose,
	  Polygon::Ptr bounds = Polygon::Ptr());
  ~ThisCar();
  
  virtual void update(const Pose2D& pose);
  virtual void update(const Pose2D& pose,
		      Path2D::Ptr path);
  
  virtual void setPose(const Pose2D& pose);
  
  virtual Path2D::ConstPtr getPath() const;
  virtual void setPath(Path2D::Ptr path);
  
  virtual const double& getPathPosition() const;
  
protected:
  /** The default driving path. */
  Path2D::Ptr _path;
  
  /** The current extension on the driving path describing the current pose on the path. */
  double _pathPos;
};

}
