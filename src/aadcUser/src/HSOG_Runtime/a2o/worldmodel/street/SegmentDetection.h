#pragma once

#include "ISegmentDetection.h"

#define DISCOUNT 0.9


namespace A2O {

class SegmentDetection : public virtual ISegmentDetection {
public:
  SegmentDetection(){};
  SegmentDetection(size_t initialCount){
    _detectionsCount = initialCount;
  };
  virtual ~SegmentDetection(){};
  
  virtual const int getDetectionsCount() const
  {
    return int(_detectionsCount);
  };
  
  virtual const size_t& getTTL() const
  {
    return _ttl;
  };
  
  virtual void decrementTTL()
  {
    if (_ttl > 0) {
      _ttl--;
    }
    _detectionsCount *= DISCOUNT;
  };
  
  virtual void incrementDetectionsCount()
  {
    _detectionsCount /= DISCOUNT;
    _detectionsCount++;
    resetTTL();
  };
  
  virtual void resetTTL()
  {
    _ttl = 50;
  };
  
protected:
  float _detectionsCount = 0;
  
  size_t _ttl = 50;
};

}
