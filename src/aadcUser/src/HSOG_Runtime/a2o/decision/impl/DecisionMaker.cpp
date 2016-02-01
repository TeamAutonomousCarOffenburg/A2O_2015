#include "DecisionMaker.h"
#include "maneuver/IManeuverConstants.h"


using namespace A2O;

DecisionMaker::DecisionMaker(ICarModel::Ptr carModel,
			     IWorldModel::Ptr worldModel,
			     IManeuverMapPtr _maneuvers)
      : _carModel(carModel), _worldModel(worldModel), _maneuvers(_maneuvers)
{
  _currentManeuver = _maneuvers->at(MANEUVER::GET_READY);
  _numberOfDecisions = 0;
}

DecisionMaker::~DecisionMaker()
{

}

const bool DecisionMaker::decide()
{
  if(!isReady()){
      _currentManeuver = _maneuvers->at(MANEUVER::GET_READY);
  }  
  else if(_numberOfDecisions < 2500){
     _currentManeuver = _maneuvers->at(MANEUVER::DRIVE_VIRTUAL_LANE);
  } else {
     _currentManeuver = _maneuvers->at(MANEUVER::HALT);
  }
  // Endlessly perform the current maneuver
  _currentManeuver->perform();

  _numberOfDecisions++;
  return true;
}

const bool DecisionMaker::isReady()
{
  return _carModel->isInitialized() && _worldModel->isInitialized();
}

const bool DecisionMaker::shouldRun()
{
  return _worldModel->getJuryAction() == JuryAction::START;
}
