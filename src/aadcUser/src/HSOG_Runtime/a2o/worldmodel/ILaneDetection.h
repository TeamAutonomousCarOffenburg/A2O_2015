 #pragma once

#include <boost/smart_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "carmodel/impl/CarModel.h"
#include "worldmodel/lanedetection/LineDetector.h"


namespace A2O {

/**
 * Interface for the lane detection.
 *
 */
class ILaneDetection {
public:
  virtual ~ILaneDetection(){};

  virtual void notifyPossibleNewFrame() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void setFrame(cv::Mat& frame, bool initialAlign) = 0;

  virtual void detectLanes() = 0;
  virtual LineDetector::Ptr getLaneResult() = 0;

  typedef boost::shared_ptr<ILaneDetection> Ptr;
  typedef boost::shared_ptr<const ILaneDetection> ConstPtr;
};

}
