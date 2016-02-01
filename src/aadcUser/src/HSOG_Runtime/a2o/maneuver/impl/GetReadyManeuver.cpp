#include "GetReadyManeuver.h"
#include "AADCCar.h"


using namespace A2O;

GetReadyManeuver::GetReadyManeuver(ICarModel::Ptr carModel,
				   IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::GET_READY, carModel, worldModel)
{

}

GetReadyManeuver::~GetReadyManeuver()
{

}

const ManeuverStatus GetReadyManeuver::getStatus() const
{
	return ManeuverStatus::STARTUP;
}

void GetReadyManeuver::perform()
{
  // Stop main motor and turn steering straight
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(0);
  _carModel->getServoDrive(AADC_Car::STEERING_SERVO)->setPosition(0);
  
  // Indicate stopping
  _carModel->getLight(AADC_Car::BREAK_LIGHTS)->turnOn();
  
  // Activate normal lighting
  _carModel->getLight(AADC_Car::HEAD_LIGHTS)->turnOn();
  _carModel->getLight(AADC_Car::BACK_LIGHTS)->turnOn();
  _carModel->getLight(AADC_Car::INDICATOR_LIGHTS_LEFT)->turnOff();
  _carModel->getLight(AADC_Car::INDICATOR_LIGHTS_RIGHT)->turnOff();
}
