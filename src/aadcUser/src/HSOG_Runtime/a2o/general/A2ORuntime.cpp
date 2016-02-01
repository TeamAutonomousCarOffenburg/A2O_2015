#include "A2ORuntime.h"

using namespace A2O;

A2ORuntime::A2ORuntime(IComponentFactory::ConstPtr factory) : _init(false), _factory(factory)
{

}

A2ORuntime::~A2ORuntime()
{

}

void A2ORuntime::init()
{
  // Check for unique initialization
  if (_init) {
    return;
  } else {
    _init = true;
  }
    
  // Create Meta Model
  _carMetaModel = _factory->createCarMetaModel();
  
  // Create Protocol layer
  _action = _factory->createAction(_carMetaModel);
  _perception = _factory->createPerception(_carMetaModel);
  
  // Create Model layer
  _carModel = _factory->createCarModel(_carMetaModel);
  
  _laneDetection = _factory->createLanedetection();
  _signDetection = _factory->createSignDetection();
  _objectDetection = _factory->createObjectDetection();

  
  _worldModel = _factory->createWorldModel(_carModel,_laneDetection, _signDetection, _objectDetection);
  


  
  // Create Decision & Skill layer
  _maneuverMap = _factory->createManeuvers(_carModel, _worldModel);
  _decisionMaker = _factory->createDecisionMaker(_carModel, _worldModel, _maneuverMap);

  _logger = _factory->createEventLogger();
}

ICarMetaModel::Ptr A2ORuntime::getCarMetaModel()
{
  return _carMetaModel;
}

IAction::Ptr A2ORuntime::getAction()
{
  return _action;
}

IPerception::Ptr A2ORuntime::getPerception()
{
  return _perception;
}

ICarModel::Ptr A2ORuntime::getCarModel()
{
  return _carModel;
}

IWorldModel::Ptr A2ORuntime::getWorldModel()
{
  return _worldModel;
}

IObjectDetection::Ptr A2ORuntime::getObjectDetection()
{
  return _objectDetection;
}

IManeuverMapPtr A2ORuntime::getManeuvers()
{
  return _maneuverMap;
}

IDecisionMaker::Ptr A2ORuntime::getDecisionMaker()
{
  return _decisionMaker;
}

IEventLogger::Ptr A2ORuntime::getEventLogger()
{
	return _logger;
}

const bool A2ORuntime::update()
{
  // Progress Perception
  if (!_perception->nextPerceptorMap()) {
    return false;
  }
  
  // Update models
  _carModel->update(_perception);
  _worldModel->update(_perception);
  
  return true;
}

const bool A2ORuntime::act()
{
  // Make a decision and perform a maneuver
  _decisionMaker->decide();
  
  // Reflect new actions to action component
  _carModel->reflectActions(_action);
  
  return true;
}

void A2ORuntime::start() {
  _worldModel->start();
  _logger->start();
}

void A2ORuntime::stop() {
  _worldModel->stop();
  _logger->stop();
}
