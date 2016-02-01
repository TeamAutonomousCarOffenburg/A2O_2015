#pragma once

#include <opencv/cv.h>
#include <boost/smart_ptr.hpp>

#include "worldmodel/signdetection/SignResult.h"

namespace A2O {

/**
 * Interface for the sign detection.
 *
 */
class ISignDetection {
public:
  
  virtual ~ISignDetection(){};

  virtual void notifyPossibleNewFrame() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void setFrame(cv::Mat& frame) = 0;

  virtual SignResult::Ptr getSignResult()=0;

  typedef boost::shared_ptr<ISignDetection> Ptr;
  typedef boost::shared_ptr<const ISignDetection> ConstPtr;
};

}
