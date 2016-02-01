#pragma once

#include "decision/IDecisionMaker.h"
#include "carmodel/ICarModel.h"
#include "worldmodel/IWorldModel.h"
#include "maneuver/IManeuver.h"
#include "decision/impl/DecisionMaker.h"

#include <map>


namespace A2O {


class CalibrationDecisionMaker : public DecisionMaker, public virtual IDecisionMaker{
public:
  CalibrationDecisionMaker(ICarModel::Ptr carModel,
		IWorldModel::Ptr worldModel,
		IManeuverMapPtr maneuvers);
  virtual ~CalibrationDecisionMaker();
  
  virtual const bool decide();
  

};

}
