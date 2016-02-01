#pragma once

#include "VisibleObject.h"
#include <worldmodel/IMovableObject.h>

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Base-representation for all movable objects.
 *
 * \author Stefan Glaser
 */
class MovableObject : public VisibleObject, public virtual IMovableObject {
public:
  typedef boost::shared_ptr<MovableObject> Ptr;
  typedef boost::shared_ptr<const MovableObject> ConstPtr;
  
  MovableObject(const std::string& name,
		const Pose2D& pose);
  ~MovableObject();
  
  virtual void update(const Pose2D& pose);
  
  virtual const Eigen::Vector2d& getSpeed() const;
  
protected:
  /** The speed vector of this object (lenght --> meters per second) */
  Eigen::Vector2d _speed;
};

}
