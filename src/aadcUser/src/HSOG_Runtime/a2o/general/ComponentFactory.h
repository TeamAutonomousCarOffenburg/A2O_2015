#pragma once

#include "IComponentFactory.h"

namespace A2O {

/**
 * The ComponentFactory is responsible for providing all concrete component instances during initialization.
 * 
 * \author Stefan Glaser
 */
class ComponentFactory: public virtual IComponentFactory {
public:
  ComponentFactory();
  virtual ~ComponentFactory();
  
  /** Factory method to create the meta model of the car.
   *
   * \returns a new Perception
   */
  virtual ICarMetaModel::Ptr createCarMetaModel() const;
  
  /** Factory method to create a Action.
   *
   * \returns a new Action
   */
  virtual IAction::Ptr createAction(ICarMetaModel::Ptr carMetaModel) const;
  
  /** Factory method to create a Perception.
   *
   * \returns a new Perception
   */
  virtual IPerception::Ptr createPerception(ICarMetaModel::Ptr carMetaModel) const;
  
  /** Factory method to create a CarModel.
   *
   * \returns a new CarModel
   */
  virtual ICarModel::Ptr createCarModel(ICarMetaModel::Ptr carMetaModel) const;
  
  /** Factory method to create a WorldModel.
   *
   * \returns a new WorldModel
   */
  virtual IWorldModel::Ptr createWorldModel(ICarModel::Ptr carModel, ILaneDetection::Ptr laneDetection, ISignDetection::Ptr signDetection,
		IObjectDetection::Ptr objectDetection) const;
  
  virtual IManeuverMapPtr createManeuvers(ICarModel::Ptr carModel,
					  IWorldModel::Ptr WorldModel) const;
  
  /** Factory method to create a DecisionMaker.
   *
   * \returns a new DecisionMaker
   */
  virtual IDecisionMaker::Ptr createDecisionMaker(ICarModel::Ptr carModel,
						  IWorldModel::Ptr WorldModel,
						  IManeuverMapPtr maneuvers) const;

  /** Factory method to create a LineDetection.
   *
   * \returns a new LineDetection
   */
  virtual ILaneDetection::Ptr createLanedetection() const;
  
  
  /** Factory method to create a ObjectDetection.
   *
   * \returns a new ObjectDetection
   */
  virtual IObjectDetection::Ptr createObjectDetection() const;

  virtual ISignDetection::Ptr createSignDetection() const;

  virtual IEventLogger::Ptr createEventLogger() const;
};

}
