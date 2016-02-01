#pragma once

#include <worldmodel/ILaneDetection.h>
#include <utils/logger/EventLogger.h>
#include <worldmodel/lanedetection/LoggingLaneDetectionModel.h>

namespace A2O{

class LaneDetectionLogDecorator: public virtual ILaneDetection
{
  public:
  
  LaneDetectionLogDecorator(ILaneDetection::Ptr laneDetection, std::string logfile)
  {
    _laneDetection = laneDetection;
    _logfile = logfile;
  }
  
  virtual void notifyPossibleNewFrame(){
    _laneDetection->notifyPossibleNewFrame();
  }
  virtual void start(){
    _laneDetection->start(); 
  }
  virtual void stop(){
    _laneDetection->stop();
  }
  virtual void setFrame(cv::Mat& frame){
      _laneDetection->setFrame(frame);
  }
  virtual void detectLanes(){
      _laneDetection->detectLanes();
  }
  
  
private:
  ILaneDetection::Ptr _laneDetection;
  std::string _logfile;
  
  
};
}
