#include "DriveVirtualLane.h"
#include "utils/geometry/Geometry.h"

#define VIRTUAL_TARGET_DISTANCE 1.0

using namespace A2O;


DriveVirtualLane::DriveVirtualLane(ICarModel::Ptr carModel,
				   IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::DRIVE_VIRTUAL_LANE, carModel, worldModel)
{
}

DriveVirtualLane::~DriveVirtualLane()
{
}

const ManeuverStatus DriveVirtualLane::getStatus() const
{
	return ManeuverStatus::RUNNING;
}


void DriveVirtualLane::perform()
{
  Path2D::ConstPtr path = _worldModel->getThisCar()->getPath();
  Pose2D carPose = _worldModel->getThisCar()->getPose();
  double length = path->getPositionOnPath(carPose.getPosition());
  Pose2D targetPose = path->interpolateWayPose(length + VIRTUAL_TARGET_DISTANCE);
  
  _worldModel->setVirtualWayPoint(targetPose.getPosition());
  Pose2D localPose = carPose / targetPose;
  
  // Set actions
  _carModel->steerDoubleArcTo(localPose);
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(25);

  // set indicators 
  Segment::Ptr currentSegment = _worldModel->getCurrentWorldState()->getCurrentSegment();
  if(currentSegment && currentSegment->isStraight() && currentSegment->hasIntendedOption())
  {
	SegmentLink::Ptr intendedLink = currentSegment->getIntendedOption();
	Pose2D toNextSegment = carPose / intendedLink->getPose();

	if(intendedLink->hasSegmentAfter())
	{
		Segment::Ptr next = intendedLink->getSegmentAfter();
		if(toNextSegment.x() < 0.5 && next->isCrossing() && next->hasIntendedOption())
		{
				SegmentLink::Ptr intendedLinkNext = next->getIntendedOption();
				if(intendedLinkNext->isRightOption())
				{
					_carModel->getLight(AADC_Car::INDICATOR_LIGHTS_RIGHT)->turnOn();
				}
				else if(intendedLinkNext->isLeftOption())
				{
					_carModel->getLight(AADC_Car::INDICATOR_LIGHTS_RIGHT)->turnOn();
				}
		}
	}
	else
	{
		_carModel->getLight(AADC_Car::INDICATOR_LIGHTS_RIGHT)->turnOff();
		_carModel->getLight(AADC_Car::INDICATOR_LIGHTS_RIGHT)->turnOff();
	}
  }
}



/*
void DriveVirtualLane::perform()
{
  Pose2D currentPose = _worldModel->getPose();
  
  Line2D example(0, 0, Angle(M_PI / 2) ,3);
  Angle result;
  getAngle(currentPose, &example, result);
  _carModel->getServoDrive(AADC_Car::STEERING_SERVO)->setPosition(-result.deg());
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(25);
  std::cout << "steering angle: " << -result.deg() << " currentPose angle: " << currentPose.getAngle().deg() << std::endl;

}

void DriveVirtualLane::getAngle(Pose2D& currentPose, IManeuverGeometry* geometry, Angle& result)
{
  // target is in front of the car in 0.5m distance
  Eigen::Vector2d virtualTarget = currentPose.getPosition() + currentPose.getAngle().getVector(VIRTUAL_TARGET_DISTANCE);
  // offste of the virtual target
  Pose2D offset = geometry->getClosestPose(virtualTarget);
  // check direction of the offset. if wrong change it
  if(std::fabs( offset.getAngle().rad() - currentPose.getAngle().rad()) > M_PI * 0.5)
  {
    offset.setAngle(offset.getAngle().opposite());
  }
  // distance of virtual target to offset
  double distance = Geometry::getDistance<double, 2>(virtualTarget, offset.getPosition());
  // triangle calculation
  Angle steeringAngle(atan2(distance, VIRTUAL_TARGET_DISTANCE));
  result = steeringAngle;
}*/


// void DriveVirtualLane::calcVitualLane(Pose2D* fromPose, Pose2D* toPose)
// {
//   
//   // Anlge almost equal (less than PI/8)
//   if(std::abs(fromPose->getDeltaAngle(toPose->getAngle()).rad()) < M_PI/8)
//   {
//     // triangle calculation
//     //           cC		----
//     //          /|\		bBcC
//     //	      a/ |h\b 
//     //     bB /__|__\ aA (=m)   c = r
//     //         p c q
//     Eigen::Vector2d bB = toPose->getPosition() - fromPose->getPosition();
//     double a = Geometry::getDistance<double, 2>(toPose->getPosition(), fromPose->getPosition());
//     Angle gamme = Angle::to(bB) - fromPose->getAngle();
//     
//     double h =  cos(gamme.rad()) * a;
//     double p = sqrt(a * a - h * h);
//     double q = (h * h) / p;
//     double r = p + q;
//     
//     // calculate if origin is left or right of the pose
//     Eigen::Vector2d tmp = toPose->getPosition() - fromPose->getPosition(); 
//     Angle relativAngle = Angle::to(tmp);
//     relativAngle -= fromPose->getAngle();
//     
//     // calculate circles origin
//     Eigen::Vector2d _origin;
//     Angle rotationAngle;
//     
// 
//     if(relativAngle.rad() > 0)
//       rotationAngle = fromPose->getAngle() + Angle(M_PI);
//     else
//       rotationAngle = fromPose->getAngle() + Angle(M_PI);
//     
//     Eigen::Vector2d originOffsetTmp(r,0);
//     Eigen::Vector2d originOffset(0, 0);
//     Angle::rotate(originOffsetTmp, rotationAngle.rad(), originOffset);
//     
//     _origin = originOffset + fromPose->getPosition();
// 
//   }
// }
