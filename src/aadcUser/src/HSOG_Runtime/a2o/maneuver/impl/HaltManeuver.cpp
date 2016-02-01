#include "HaltManeuver.h"
#include "AADCCar.h"


using namespace A2O;

A2O::HaltManeuver::HaltManeuver(ICarModel::Ptr carModel,
				IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::HALT, carModel, worldModel)
{

}

A2O::HaltManeuver::~HaltManeuver()
{

}

void A2O::HaltManeuver::perform()
{
  // Stop main motor
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(0);
  
  // Indicate stopping
  _carModel->getLight(AADC_Car::BREAK_LIGHTS)->turnOn();
  
  // Activate hazard lights
  _carModel->getLight(AADC_Car::INDICATOR_LIGHTS_LEFT)->turnOn();
  _carModel->getLight(AADC_Car::INDICATOR_LIGHTS_RIGHT)->turnOn();
  
  // Activate all other lights :)
  _carModel->getLight(AADC_Car::HEAD_LIGHTS)->turnOn();
  _carModel->getLight(AADC_Car::BACK_LIGHTS)->turnOn();
  
  _status = ManeuverStatus::COMPLETE;
}
