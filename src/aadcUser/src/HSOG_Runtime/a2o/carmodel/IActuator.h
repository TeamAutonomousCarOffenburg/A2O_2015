#pragma once

#include <string>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for all known actuators of the system.
 * Each actuator specifies a unique name, perceptor-name and effector-name.
 * 
 * \author Stefan Glaser
 */
class IActuator {
public:
  typedef boost::shared_ptr<IActuator> Ptr;
  typedef boost::shared_ptr<const IActuator> ConstPtr;

  virtual ~IActuator(){};
  
  /** Retrieve the name of the actuator.
   *
   * \returns The actuator name.
   */
  virtual const std::string& getName() const = 0;
  
  /** Retrieve the time this actuator received the latest update.
   *
   * \returns The time of the latest actuator update
   */
  virtual const long& getLastMeasurementTime() const = 0;
  
  /** \brief Check if this actuator is properly initialized or not.
    *
    * \returns true, if this actuator is properly initialized, false otherwise
    */
  virtual const bool isInitialized() const = 0;
};

}
