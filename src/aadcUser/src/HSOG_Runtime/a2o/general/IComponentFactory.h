#pragma once

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
#include "utils/logger/IEventLogger.h"

namespace A2O {

/**
 * The IComponentFactory provides an interface for concrete component factories.
 *
 * \author Stefan Glaser
 */
class IComponentFactory {
public:
  typedef boost::shared_ptr<IComponentFactory> Ptr;
  typedef boost::shared_ptr<const IComponentFactory> ConstPtr;

  virtual ~IComponentFactory(){};
  
  /** Factory method to create the meta model of the car.
   *
   * \returns a new Perception
   */
  virtual ICarMetaModel::Ptr createCarMetaModel() const = 0;
  
  /** Factory method to create a Action.
   *
   * \returns a new Action
   */
  virtual IAction::Ptr createAction(ICarMetaModel::Ptr carMetaModel) const = 0;
  
  /** Factory method to create a Perception.
   *
   * \returns a new Perception
   */
  virtual IPerception::Ptr createPerception(ICarMetaModel::Ptr carMetaModel) const = 0;
  
  /** Factory method to create a CarModel.
   *
   * \returns a new CarModel
   */
  virtual ICarModel::Ptr createCarModel(ICarMetaModel::Ptr carMetaModel) const = 0;
  
  /** Factory method to create a WorldModel.
   *
   * \returns a new WorldModel
   */
  virtual IWorldModel::Ptr createWorldModel(ICarModel::Ptr carModel, ILaneDetection::Ptr laneDetection, ISignDetection::Ptr signDetection, IObjectDetection::Ptr objectDetection) const = 0;
  
  virtual IManeuverMapPtr createManeuvers(ICarModel::Ptr carModel,
					  IWorldModel::Ptr WorldModel) const = 0;
					  
  virtual IObjectDetection::Ptr createObjectDetection() const = 0;
  
  /** Factory method to create a DecisionMaker.
   *
   * \returns a new DecisionMaker
   */
  virtual IDecisionMaker::Ptr createDecisionMaker(ICarModel::Ptr carModel,
						  IWorldModel::Ptr WorldModel,
						  IManeuverMapPtr maneuvers) const = 0;

  /** Factory method to create a LaneDetection.
   *
   * \returns a new LaneDetection
   */
  virtual ILaneDetection::Ptr createLanedetection() const = 0;

  /** Factory method to create a SignDetection.
   *
   * \returns a new SignDetection
   */
  virtual ISignDetection::Ptr createSignDetection() const = 0;

  /** Factory method to create a EventLogger.
   *
   * \returns a new EventLogger
   */
  virtual IEventLogger::Ptr createEventLogger() const = 0;

};

}
