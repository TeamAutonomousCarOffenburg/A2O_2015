#include "CalibrationDecisionMaker.h"
#include "maneuver/IManeuverConstants.h"
#include "maneuver/impl/Calibration.h"


using namespace A2O;

CalibrationDecisionMaker::CalibrationDecisionMaker(ICarModel::Ptr carModel,
			     IWorldModel::Ptr worldModel,
			     IManeuverMapPtr _maneuvers)
      : DecisionMaker(carModel, worldModel, _maneuvers)
{
  
}

CalibrationDecisionMaker::~CalibrationDecisionMaker()
{

}

const bool CalibrationDecisionMaker::decide()
{
  
  if(_numberOfDecisions < 250 ){
      _currentManeuver = _maneuvers->at(MANEUVER::GET_READY);
  }
  else if(_numberOfDecisions < 2500){
     _currentManeuver = _maneuvers->at(MANEUVER::CALIBRATION);
     
    
    int speed = 30;
    double radius = 1.0;
     
     Calibration::Ptr cali = boost::dynamic_pointer_cast<Calibration>(_currentManeuver);

    cali->setRadius(radius);
    cali->setSpeed(speed);
     

    
     
  } else {
     _currentManeuver = _maneuvers->at(MANEUVER::HALT);
  }
  // Endlessly perform the current maneuver
  _currentManeuver->perform();
  
  
  
  _numberOfDecisions++;
  return true;
}
