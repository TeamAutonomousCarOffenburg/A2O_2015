#include "DriveWayPoints.h"
#include <AADCCar.h>
#include "utils/geometry/drive/Driveable.h"
#include "utils/geometry/Geometry.h"
#include <vector>


using namespace A2O;
using namespace std;
using namespace Eigen;

#define OFFSET 0.15


DriveWayPoints::DriveWayPoints(ICarModel::Ptr carModel,
			       IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::DRIVE_WAY_POINTS, carModel, worldModel)
{}

DriveWayPoints::~DriveWayPoints()
{}

void DriveWayPoints::perform()
{
  driveWayPoint();
}

void DriveWayPoints::driveWayPoint(int skip /* = 0 */)
{
  Pose2D carPose = _worldModel->getThisCar()->getPose();
  Pose2D nextWayPoint = getNextWayPoint(skip);
  
  // calculate different geometries
  Driveable line = DriveGeometry::getLine(carPose, nextWayPoint);
  Driveable circle = DriveGeometry::getCircle(carPose, nextWayPoint);
  Driveable lineBeforeCircle = DriveGeometry::getLineBeforeCircleGeometry(carPose, nextWayPoint);
  Driveable circleBeforeLine = DriveGeometry::getCircleBeforeLineGeometry(carPose, nextWayPoint);
  Driveable sCurve = DriveGeometry::getSCurveBeforeLineGeometry(carPose, nextWayPoint);
  
  //   evaluate geometries
  if(line.isValid())
  {
    cout << "DriveWayPoints: follow line.";
    followGeometry(&line, carPose, nextWayPoint);  
  }
  else if(circle.isValid())
  {
     cout << "DriveWayPoints: follow circle.";
     followGeometry(&circle, carPose, nextWayPoint);
  }
  else  if(lineBeforeCircle.isValid())
  {
    cout << "DriveWayPoints: follow line before circle.";
    followGeometry(&lineBeforeCircle, carPose, nextWayPoint);
  }
  else if(circleBeforeLine.isValid())
  {
    cout << "DriveWayPoints: follow circle before line.";
    followGeometry(&circleBeforeLine, carPose, nextWayPoint);
  } 
  else if(sCurve.isValid())
  {
    cout << "DriveWayPoints: follow s curve.";
    followGeometry(&sCurve, carPose, nextWayPoint);
  }
  else{
    // next way point is not reachable by standart geometry. Skip way point and go on.
    cout << "DriveWayPoints: skipping way point -> ";
    driveWayPoint(skip + 1);
  }
}


const Pose2D DriveWayPoints::getNextWayPoint(int skip /* = 0 */) const
{
  const std::vector<Pose2D>& wayPoints = _worldModel->getThisCar()->getPath()->getWayPoints();
  Pose2D carPose = _worldModel->getThisCar()->getPose();
  Pose2D pose = carPose * Pose2D(OFFSET, 0);

  if (wayPoints.size() == 0) {
    return pose;
  }

  size_t idx = 0;
  Pose2D relativeWayPoint;

  do {
    relativeWayPoint = pose / wayPoints[idx];
    idx++;
  } while (relativeWayPoint.x() < 0 && idx + skip < wayPoints.size());
  
  if (relativeWayPoint.x() < 0 || idx + skip > wayPoints.size()) {
    // The final way point is still behind us, so just take a pose in fron of us
    return pose;
  }
  
  return wayPoints[idx - 1 + skip];
}

void DriveWayPoints::followGeometry(IManeuverGeometry* geometry, Pose2D& carPose, Pose2D& nextWayPoint)
{ 
  _worldModel->setVirtualWayPoint(nextWayPoint.getPosition());  
  _worldModel->setVirtualTrail(geometry->getTrail(carPose.getPosition(), nextWayPoint.getPosition()));
  Angle angle = getSteeringAngle(geometry, carPose);
  cout << " New steering angle (degrees): " << angle.deg() << endl;
  _carModel->getServoDrive(AADC_Car::STEERING_SERVO)->setPosition(angle.deg());
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(_speed);
}

Angle DriveWayPoints::getSteeringAngle(IManeuverGeometry* geometry, Pose2D& carPose)
{
  Vector2d virtualPoint = carPose.getPosition() + carPose.getAngle().getVector(OFFSET);
  Pose2D globalVirtualPose = geometry->getClosestPose(virtualPoint);
  Pose2D target = carPose / globalVirtualPose;
  return Angle::to(target.x(), target.y());
}





