#include "ComponentFactory.h"

#include "action/impl/Action.h"
#include "perception/impl/Perception.h"
#include "carmodel/impl/CarModel.h"
#include "worldmodel/impl/WorldModel.h"
#include "worldmodel/lanedetection/openCvLane.h"

#ifdef ARUCO_FOUND
#include "worldmodel/signdetection/BitCodeSignDetection.h"
#endif

#include "worldmodel/signdetection/DummySignDetection.h"

#include "decision/impl/DecisionMaker.h"
#include "decision/impl/AADCDecisionMaker.h"
#include "meta/impl/AADCCarMetaModel.h"

#include "maneuver/impl/HaltManeuver.h"
#include "maneuver/impl/GetReadyManeuver.h"
#include "maneuver/impl/IsReadyManeuver.h"
#include "maneuver/impl/FollowLaneManeuver.h"
#include "maneuver/impl/DriveVirtualLane.h"
#include "maneuver/impl/DriveWayPoints.h"
#include "maneuver/impl/DriveWayPointsStraight.h"
#include "maneuver/impl/OneMeter.h"
#include "maneuver/impl/Calibration.h"
#include "maneuver/impl/JuryStatus.h"

#include "worldmodel/objectdetection/ObjectDetection.h"
#include "worldmodel/IObjectDetection.h"

#include "utils/logger/EventLogger.h"
#include "utils/logger/DummyEventLogger.h"

using namespace A2O;

ComponentFactory::ComponentFactory()
{

}

ComponentFactory::~ComponentFactory()
{

}

ICarMetaModel::Ptr ComponentFactory::createCarMetaModel() const
{
  return boost::make_shared<AADCCarMetaModel>();
}

IAction::Ptr ComponentFactory::createAction(ICarMetaModel::Ptr carMetaModel) const
{
  return boost::make_shared<Action>(carMetaModel);
}

IPerception::Ptr ComponentFactory::createPerception(ICarMetaModel::Ptr carMetaModel) const
{
  return boost::make_shared<Perception>();
}

ICarModel::Ptr ComponentFactory::createCarModel(ICarMetaModel::Ptr carMetaModel) const
{
  return boost::make_shared<CarModel>(carMetaModel);
}

IWorldModel::Ptr ComponentFactory::createWorldModel(ICarModel::Ptr carModel,
						    ILaneDetection::Ptr laneDetection,
						    ISignDetection::Ptr signDetection, 
						    IObjectDetection::Ptr objectDetection) const
{
  return boost::make_shared<WorldModel>(carModel, laneDetection, signDetection, objectDetection, false);
}

ILaneDetection::Ptr ComponentFactory::createLanedetection() const
{
  return boost::make_shared<LaneDetection>();
}

IObjectDetection::Ptr ComponentFactory::createObjectDetection() const
{
  return boost::make_shared<ObjectDetection>();
}

IEventLogger::Ptr ComponentFactory::createEventLogger() const
{
  return boost::make_shared<DummyEventLogger>(); 
}

IManeuverMapPtr ComponentFactory::createManeuvers(ICarModel::Ptr carModel,
						  IWorldModel::Ptr WorldModel) const
{
  IManeuverMapPtr maneuverMap(new std::map<std::string, IManeuver::Ptr>());
  
  // Add maneuvers
  IManeuver::Ptr maneuver(boost::make_shared<HaltManeuver>(carModel, WorldModel));
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<GetReadyManeuver>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<IsReadyManeuver>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<FollowLaneManeuver>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<DriveVirtualLane>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));

  maneuver = boost::make_shared<OneMeter>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<Calibration>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<DriveWayPoints>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<DriveWayPointsStraight>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  maneuver = boost::make_shared<JuryStatus>(carModel, WorldModel);
  maneuverMap->insert(make_pair(maneuver->getName(), maneuver));
  
  
  return maneuverMap;
}

IDecisionMaker::Ptr ComponentFactory::createDecisionMaker(ICarModel::Ptr carModel,
							  IWorldModel::Ptr WorldModel,
							  IManeuverMapPtr maneuvers) const
{
  return boost::make_shared<AADCDecisionMaker>(carModel, WorldModel, maneuvers);
}

ISignDetection::Ptr ComponentFactory::createSignDetection() const
{
#ifdef ARUCO_FOUND
	return boost::make_shared<BitCodeSignDetection>();
#else
	return boost::make_shared<DummySignDetection>();
#endif

}
