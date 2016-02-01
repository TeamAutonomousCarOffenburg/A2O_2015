#include "JuryStatus.h"
#include "AADCCar.h"

#include <iostream>


using namespace A2O;

JuryStatus::JuryStatus(ICarModel::Ptr carModel,
				IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::SENDSTATUS_TO_JURY, carModel, worldModel)
{

}

JuryStatus::~JuryStatus()
{

}

void JuryStatus::perform()
{
	_carModel->getManeuverStatusActuator(AADC_Car::DRIVE_STATUS)->setStatus(_status);
	_carModel->getManeuverStatusActuator(AADC_Car::DRIVE_STATUS)->setCurrentManeuver(_currentManeuver);
}

void JuryStatus::setStatus(const ManeuverStatus& status)
{
     _status = status;
}

void JuryStatus::setCurrentManeuver(int currentManeuver)
{
	_currentManeuver = currentManeuver;
}
