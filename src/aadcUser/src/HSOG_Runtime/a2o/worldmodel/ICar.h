#pragma once

#include "IMovableObject.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * General Interface for all car objects.
 *
 * \author Stefan Glaser
 */
class ICar : public virtual IMovableObject {
public:
  typedef boost::shared_ptr<ICar> Ptr;
  typedef boost::shared_ptr<const ICar> ConstPtr;
  
  ~ICar(){};
};

}
