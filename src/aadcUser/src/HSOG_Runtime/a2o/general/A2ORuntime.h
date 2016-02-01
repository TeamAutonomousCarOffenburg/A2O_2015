#pragma once

#include "IComponentFactory.h"

#include "meta/ICarMetaModel.h"
#include "action/IAction.h"
#include "perception/IPerception.h"
#include "carmodel/ICarModel.h"
#include "worldmodel/IWorldModel.h"
#include "worldmodel/ILaneDetection.h"
#include "worldmodel/ISignDetection.h"
#include "maneuver/IManeuver.h"
#include "decision/IDecisionMaker.h"
#include "worldmodel/IObjectDetection.h"

namespace A2O {

/**
 * The A2ORuntime is the central component managing the application flow.
 * 
 * \author Stefan Glaser
 */
class A2ORuntime {
public:
  A2ORuntime(IComponentFactory::ConstPtr factory);
  virtual ~A2ORuntime();
  
  virtual A2O::ICarMetaModel::Ptr getCarMetaModel();
  virtual A2O::IAction::Ptr getAction();
  virtual A2O::IPerception::Ptr getPerception();
  virtual A2O::ICarModel::Ptr getCarModel();
  virtual A2O::IWorldModel::Ptr getWorldModel();
  virtual A2O::IObjectDetection::Ptr getObjectDetection();
  virtual A2O::IManeuverMapPtr getManeuvers();
  virtual A2O::IDecisionMaker::Ptr getDecisionMaker();
  
 virtual A2O::IEventLogger::Ptr getEventLogger(); 

  /** \brief Initialize all runtime components.
   * 
   * This method has to be called once dusing application start.
   * It is responsible for creating the several runtime components of the system.
   */
  virtual void init();
  
  /** \brief Called from outside to update the system.
   * 
   * The update method tries to fetch a new perception-map and, if successful,
   * updates all components of the model layer.
   * 
   * \return success
   */
  virtual const bool update();
  
  /** \brief Called from outside to let the system make a decision and perform some actions.
   * 
   * The act method calls the decision maker to make a new decision and takes care
   * of reflecting the new actions to the action component.
   * 
   * \return success
   */
  virtual const bool act();

  virtual void start();
  virtual void stop();

protected:
  /** Initialization flag. */
  bool _init;

  /** Logger to log events */
  A2O::IEventLogger::Ptr _logger;

  A2O::IComponentFactory::ConstPtr _factory;

  /** Reference to the car meta model. */
  A2O::ICarMetaModel::Ptr _carMetaModel;
  
  /** Reference to the action component. */
  A2O::IAction::Ptr _action;
  
  /** Reference to the perception component. */
  A2O::IPerception::Ptr _perception;
  
  /** Reference to the car model component. */
  A2O::ICarModel::Ptr _carModel;
  
  /** Reference to the world model component. */
  A2O::IWorldModel::Ptr _worldModel;
  
  /** Reference to the map of available maneuvers. */
  A2O::IManeuverMapPtr _maneuverMap;
  
  /** Reference to the decision maker component. */
  A2O::IDecisionMaker::Ptr _decisionMaker;
  
  /** Reference to Lanedetection. */
  A2O::ILaneDetection::Ptr _laneDetection;
  
  /** Reference to object detection */
  A2O::IObjectDetection::Ptr _objectDetection;
  
  /** Reference to sign detection */
  A2O::ISignDetection::Ptr _signDetection;
};
}
