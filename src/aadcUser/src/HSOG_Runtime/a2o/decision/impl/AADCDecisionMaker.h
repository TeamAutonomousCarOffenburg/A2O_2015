#pragma once

#include "decision/IDecisionMaker.h"
#include "carmodel/ICarModel.h"
#include "worldmodel/IWorldModel.h"
#include "maneuver/IManeuver.h"
#include "decision/impl/DecisionMaker.h"

#include <map>


namespace A2O {


class AADCDecisionMaker : public DecisionMaker, public virtual IDecisionMaker{
public:
  AADCDecisionMaker(ICarModel::Ptr carModel,
		IWorldModel::Ptr worldModel,
		IManeuverMapPtr maneuvers);
  virtual ~AADCDecisionMaker();
  
  virtual const bool decide();

private:
	void sendToJury();

	DriveInstructionManager::Ptr _mngr;

	IManeuver::Ptr _juryManeuver;
};

}
