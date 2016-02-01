#pragma once

#include <stddef.h>

namespace A2O {

class ISegmentDetection {
public:
  virtual ~ISegmentDetection(){};
  
  virtual const int getDetectionsCount() const = 0;
  
  virtual const size_t& getTTL() const = 0;
  
  virtual void decrementTTL() = 0;
};

}
