#include "OneMeter.h"
#include "AADCCar.h"


using namespace A2O;

A2O::OneMeter::OneMeter(ICarModel::Ptr carModel,
				IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::ONE_METER, carModel, worldModel)
{
	_startPose = _worldModel->getThisCar()->getPose();
	_status = ManeuverStatus::READY;
}

A2O::OneMeter::~OneMeter()
{
}

void A2O::OneMeter::perform()
{
  _carModel->steer(Angle::Zero());
  if (_worldModel->getThisCar()->getPose().x() - _startPose.x() < 1.0) {
    _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(30);
    _status = ManeuverStatus::RUNNING;
  } else {
    _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(0);
    _status = ManeuverStatus::COMPLETE;
  }
}
