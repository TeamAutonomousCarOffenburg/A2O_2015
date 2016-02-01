#pragma once
#include "lanedetection/LaneResult.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Interface for the lane detection model, that takes the result of the lanedetection.
 *
 */
class ILaneDetectionModel {
public:
  
  typedef boost::shared_ptr<ILaneDetectionModel> Ptr;
  typedef boost::shared_ptr<const ILaneDetectionModel> ConstPtr;
  
  virtual ~ILaneDetectionModel(){};

  virtual void updateLaneResult(const LaneResult& newResult) = 0;
};

}
