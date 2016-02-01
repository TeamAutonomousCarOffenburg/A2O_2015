#pragma once

#include <string>
#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Basic Interface for all effectors.
 * All effectors define a unique name and time.
 * 
 * \author Stefan Glaser
 */
class IEffector {
public:
  typedef boost::shared_ptr<IEffector> Ptr;
  typedef boost::shared_ptr<const IEffector> ConstPtr;

  virtual ~IEffector(){};
  
  virtual const std::string& getName() const = 0;
  virtual const long& getTime() const = 0;
};

}
