#pragma once

#include "decision/IDecisionMaker.h"
#include "carmodel/ICarModel.h"
#include "worldmodel/IWorldModel.h"
#include "maneuver/IManeuver.h"

#include <map>


namespace A2O {

/**
 * The DecisionMaker class acs as base class for all decision makers.
 * 
 * \author Stefan Glaser
 */
class DecisionMaker : public virtual IDecisionMaker {
public:
  DecisionMaker(ICarModel::Ptr carModel,
		IWorldModel::Ptr worldModel,
		IManeuverMapPtr maneuvers);
  virtual ~DecisionMaker();
  
  virtual const bool decide();
  
protected:
  virtual const bool isReady();
  virtual const bool shouldRun();
  
  /** The car model. */
  ICarModel::Ptr _carModel;
  
  /** The world model. */
  IWorldModel::Ptr _worldModel;
  
  /** The map od available maneuvers. */
  IManeuverMapPtr _maneuvers;
  
  /** The currently performed maneuver. */
  IManeuver::Ptr _currentManeuver;
  
  /** Decision counter. */
  unsigned long _numberOfDecisions;
};

}
