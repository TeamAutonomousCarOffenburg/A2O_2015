#include "DriveWayPointsStraight.h"
#include <AADCCar.h>

#include <vector>

using namespace A2O;


DriveWayPointsStraight::DriveWayPointsStraight(ICarModel::Ptr carModel,
			       IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::DRIVE_WAY_POINTS_STRAIGHT, carModel, worldModel)
{

}

void DriveWayPointsStraight::perform()
{
  Pose2D carPose = _worldModel->getThisCar()->getPose();
  Pose2D localTarget = carPose / getNextWayPoint();

  _carModel->steerTo(localTarget.getPosition());
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(25);
}

const Pose2D DriveWayPointsStraight::getNextWayPoint() const
{
  const std::vector<Pose2D>& wayPoints = _worldModel->getThisCar()->getPath()->getWayPoints();
  Pose2D carPose = _worldModel->getThisCar()->getPose();
  Pose2D pose = carPose * Pose2D(0.25, 0);

  if (wayPoints.size() == 0) {
    return pose;
  }

  size_t idx = 0;
  Pose2D relativeWayPoint;

  do {
    relativeWayPoint = pose / wayPoints[idx];
    idx++;
  } while (relativeWayPoint.x() < 0 && idx < wayPoints.size());
  
  if (relativeWayPoint.x() < 0 || idx > wayPoints.size()) {
    // The final way point is still behind us, so just take a pose in fron of us
    return pose;
  }
  
  return wayPoints[idx - 1];
}
