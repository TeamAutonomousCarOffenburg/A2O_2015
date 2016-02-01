#pragma once

#include <string>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for all known perceptors of the system.
 * 
 * \author Stefan Glaser
 */
class IPerceptor {
public:
  typedef boost::shared_ptr<IPerceptor> Ptr;
  typedef boost::shared_ptr<const IPerceptor> ConstPtr;
  
  virtual ~IPerceptor(){};

  /** Retrieve the name of the perceptor.
   *
   * \returns The perceptor name.
   */
  virtual const std::string& getName() const = 0;
  
  /** Retrieve the time of the measurement.
   *
   * \returns The measurement time.
   */
  virtual const long& getTime() const = 0;
};

}
