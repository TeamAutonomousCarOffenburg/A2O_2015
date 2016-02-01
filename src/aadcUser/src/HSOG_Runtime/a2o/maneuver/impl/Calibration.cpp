#include "Calibration.h"
#include "AADCCar.h"

#include <iostream>


using namespace A2O;

A2O::Calibration::Calibration(ICarModel::Ptr carModel,
				IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::CALIBRATION, carModel, worldModel)
{

}

A2O::Calibration::~Calibration()
{

}

void A2O::Calibration::perform()
{
  
  
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(_speed);
  
 
   
  if(_radius < 0.1)
  {
   _carModel->getServoDrive(AADC_Car::STEERING_SERVO)->setPosition(0);
   _status = ManeuverStatus::ERROR;
  }
  else
  {
     Angle angle = _carModel->getDriveCalculator()->calculateSteeringAngle(_radius);
     
    _carModel->getServoDrive(AADC_Car::STEERING_SERVO)->setPosition(angle.deg()); 
    std::cout << "servo: " << angle.deg() << std::endl;  
  }

}
