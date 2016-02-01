#pragma once

#include "IVisibleObject.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * General Interface for all movable objects.
 *
 * \author Stefan Glaser
 */
class IMovableObject : virtual public IVisibleObject {
public:
  typedef boost::shared_ptr<IMovableObject> Ptr;
  typedef boost::shared_ptr<const IMovableObject> ConstPtr;
  
  ~IMovableObject(){};
  
  /** Retireve the speed of this movable object.
    * \returns the speed of this object (in meter per second)
    */
  virtual const Eigen::Vector2d& getSpeed() const = 0;
};

}
