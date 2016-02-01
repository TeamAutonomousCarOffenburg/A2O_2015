#pragma once

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Interface for the signs.
 *
 */
class ISign{
public:
  virtual ~ISign(){};
  
  virtual std::string getName()=0;
  virtual void setName(std::string name)=0;

  typedef boost::shared_ptr<ISign> Ptr;
  typedef boost::shared_ptr<const ISign> ConstPtr;
};


}
