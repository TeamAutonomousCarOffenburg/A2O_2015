#pragma once

#include <chrono>
#include <ctime>

#include "general/ComponentFactory.h"
#include "utils/logger/EventLogger.h"
#include "worldmodel/impl/WorldModel.h"
#include "worldmodel/impl/WorldModelLogDecorator.h"
#include "worldmodel/lanedetection/openCvLane.h"

namespace A2O 
{

class LogComponentFactory : public ComponentFactory
{
public:
  
  LogComponentFactory(){
    _baseLogfile = generateFileName();
  }
  
  IEventLogger::Ptr createEventLogger() const override
  {
  return IEventLogger::Ptr(EventLogger::getLogger(_baseLogfile)); 
  }
  
  ILaneDetection::Ptr createLanedetection() const override
  {
  return  boost::make_shared<LaneDetection>();
  }
  
  IWorldModel::Ptr createWorldModel(ICarModel::Ptr carModel, ILaneDetection::Ptr laneDetection, ISignDetection::Ptr signDetection, IObjectDetection::Ptr objectDetection) const override
  {
	IWorldModel::Ptr model = ComponentFactory::createWorldModel(carModel, laneDetection, signDetection, objectDetection);
	return boost::make_shared<WorldModelLogDecorator>(model, _baseLogfile);

  }
  
private:
  std::string _baseLogfile;
  
  const std::string generateFileName(){
    std::stringstream ss;
    ss << "hsog_log_" << getTimeString() << ".csv";
    return ss.str();
  }
  
  const std::string getTimeString(){
     char buffer[25];
     struct tm* tm_info;

     time_t timer;
     time(&timer);
     tm_info = localtime(&timer);
     
     strftime(buffer, 25, "%Y_%m_%d_%H_%M_%S", tm_info);
     std::string timeString = std::string(buffer);
     return timeString;
  }
  
};
}
