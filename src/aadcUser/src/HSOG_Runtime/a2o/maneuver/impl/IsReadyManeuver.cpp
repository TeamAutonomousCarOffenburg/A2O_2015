#include "IsReadyManeuver.h"
#include "AADCCar.h"


using namespace A2O;

IsReadyManeuver::IsReadyManeuver(ICarModel::Ptr carModel,
				   IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::IS_READY, carModel, worldModel)
{

}

IsReadyManeuver::~IsReadyManeuver()
{
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(0);
  _carModel->getServoDrive(AADC_Car::STEERING_SERVO)->setPosition(0);
  
  // Deactivate normal lighting
  _carModel->getLight(AADC_Car::BREAK_LIGHTS)->turnOff();
  _carModel->getLight(AADC_Car::HEAD_LIGHTS)->turnOff();
  _carModel->getLight(AADC_Car::BACK_LIGHTS)->turnOff();
  _carModel->getLight(AADC_Car::WARN_LIGHTS)->turnOff();
  _carModel->getLight(AADC_Car::INDICATOR_LIGHTS_LEFT)->turnOff();
  _carModel->getLight(AADC_Car::INDICATOR_LIGHTS_RIGHT)->turnOff();

}

const ManeuverStatus IsReadyManeuver::getStatus() const
{
	return ManeuverStatus::READY;
}

void IsReadyManeuver::perform()
{
}
