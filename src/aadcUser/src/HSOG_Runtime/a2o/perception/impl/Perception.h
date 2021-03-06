#pragma once

#include "perception/IPerception.h"
#include "Perceptor.h"

#include <string>
#include <map>
#include <queue>
#include <mutex>
#include <boost/smart_ptr.hpp>

namespace A2O {
  
/**
 * The Perception class is responsible vor managing incomming perceptions.
 * 
 * \author Stefan Glaser
 */
class Perception : public virtual IPerception {
public:
  Perception();
  virtual ~Perception();
  
  virtual IPerceptor::ConstPtr getPerceptor(std::string name) const;
  virtual IDoubleValuePerceptor::ConstPtr getDoubleValuePerceptor(std::string name) const;
  virtual ICameraPerceptor::ConstPtr getCameraPerceptor(std::string name) const;
  virtual IPointCloudPerceptor::ConstPtr getPointCloudPerceptor(std::string name) const;
  virtual IGyroPerceptor::ConstPtr getGyroPerceptor(std::string name) const;
  virtual IAccelerometerPerceptor::ConstPtr getAccelerometerPerceptor(std::string name) const;
  virtual IIMUPerceptor::ConstPtr getIMUPerceptor(std::string name) const;
  virtual IJuryPerceptor::ConstPtr getJuryPerceptor(std::string name) const;
  virtual IWheelTickPerceptor::ConstPtr getWheelTickPerceptor(std::string name) const;

  virtual const bool nextPerceptorMap();
  virtual void updatePerceptors(std::map<std::string, IPerceptor::ConstPtr>& newPerceptors);
  
protected:
  std::map<std::string, IPerceptor::ConstPtr> _perceptors;
  std::deque<std::map<std::string, IPerceptor::ConstPtr>> _perceptorQueue;

  std::mutex _queueMutex;
};

}
