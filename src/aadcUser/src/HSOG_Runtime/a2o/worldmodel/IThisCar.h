#pragma once

#include "ICar.h"
#include "utils/geometry/Path2D.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * General Interface for this (our own) car.
 *
 * \author Stefan Glaser
 */
class IThisCar : virtual public IMovableObject {
public:
  typedef boost::shared_ptr<IThisCar> Ptr;
  typedef boost::shared_ptr<const IThisCar> ConstPtr;
  
  ~IThisCar(){};
  
  /** Retrieve the default driving path. */
  virtual Path2D::ConstPtr getPath() const = 0;
  
  /** Set the default driving path. */
  virtual void setPath(Path2D::Ptr path) = 0;
  
  /** Retrieve the position on the path. */
  virtual const double& getPathPosition() const = 0;
};

}
