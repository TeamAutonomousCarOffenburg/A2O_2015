#include "AADCDecisionMaker.h"
#include "maneuver/IManeuverConstants.h"
#include "maneuver/impl/JuryStatus.h"
#include "worldmodel/IDriveInstructionConstants.h"

using namespace A2O;
using namespace std;

AADCDecisionMaker::AADCDecisionMaker(ICarModel::Ptr carModel,
		IWorldModel::Ptr worldModel,
		IManeuverMapPtr _maneuvers)
      : DecisionMaker(carModel, worldModel, _maneuvers)
{
	_mngr = _worldModel->getDriveInstructionManager();
	_juryManeuver = _maneuvers->at(MANEUVER::SENDSTATUS_TO_JURY);
}

AADCDecisionMaker::~AADCDecisionMaker()
{

}

void AADCDecisionMaker::sendToJury(){
	boost::dynamic_pointer_cast<JuryStatus>(_juryManeuver)->setStatus(_currentManeuver->getStatus());
	boost::dynamic_pointer_cast<JuryStatus>(_juryManeuver)->setCurrentManeuver(_mngr->getCurrentInstructionIndex());
	_juryManeuver->perform();
}


const bool AADCDecisionMaker::decide()
{
	_numberOfDecisions++;

	if (!isReady()) {
		_currentManeuver = _maneuvers->at(MANEUVER::GET_READY);
		_currentManeuver->perform();
		return true;
	}
	
	sendToJury();

	if (!shouldRun()) {
		_currentManeuver = _maneuvers->at(MANEUVER::IS_READY);
		_currentManeuver->perform();
		return true;
	}


	if(_currentManeuver->getStatus() == ManeuverStatus::COMPLETE)
	{
	}
	
	else
	{
		_currentManeuver = _maneuvers->at(MANEUVER::DRIVE_VIRTUAL_LANE);
	}

	_currentManeuver->perform();
	return true;
}
