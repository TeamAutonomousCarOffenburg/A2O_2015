#pragma once

#include "MovableObject.h"
#include "worldmodel/ICar.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Representation for all cars.
 *
 * \author Stefan Glaser
 */
class Car : public MovableObject, public virtual ICar {
public:
  typedef boost::shared_ptr<Car> Ptr;
  typedef boost::shared_ptr<const Car> ConstPtr;
  
  Car(const std::string& name,
      const Pose2D& pose,
      Polygon::Ptr bounds = Polygon::Ptr());
  ~Car();
  
protected:
};

}
