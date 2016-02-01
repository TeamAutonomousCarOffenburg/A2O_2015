#include "OneMeter.h"
#include "AADCCar.h"


using namespace A2O;

A2O::OneMeter::BlindParking(ICarModel::Ptr carModel,
				IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::ONE_METER, carModel, worldModel)
{
	_startPose = _worldModel->getThisCar()->getPose();
	_status = ManeuverStatus::READY;
}

A2O::OneMeter::~BlindParking()
{
}

void A2O::OneMeter::perform()
{
  Path2D::ConstPtr path = _worldModel->getThisCar()->getPath();
  Pose2D carPose = _worldModel->getThisCar()->getPose();
  double length = path->getPositionOnPath(carPose.getPosition());
  Pose2D targetPose = path->interpolateWayPose(length + 1.0);
  
  _worldModel->setVirtualWayPoint(targetPose.getPosition());
  Pose2D localPose = carPose / targetPose;
  
  // Set actions
  _carModel->steerDoubleArcTo(localPose);
  
  if (_worldModel->getThisCar()->getPose().x() < 1.0) {
    _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(30);
    _status = ManeuverStatus::RUNNING;
  }
  else if(_worldModel->getThisCar()->getPose().x() < 4.0)
  {
    _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(50);
    _status = ManeuverStatus::RUNNING;
  }
  else if(_worldModel->getThisCar()->getPose().x() < 9.0)
  {
    _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(30);
    _status = ManeuverStatus::RUNNING; 
  } else {
    _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(0);
    _status = ManeuverStatus::COMPLETE;
  }
}
