#include "FollowLaneManeuver.h"
#include <iostream>

using namespace A2O;

FollowLaneManeuver::FollowLaneManeuver(ICarModel::Ptr carModel,
				       IWorldModel::Ptr worldModel)
      : Maneuver(MANEUVER::FOLLOW_LANE, carModel, worldModel)
{

}

FollowLaneManeuver::~FollowLaneManeuver()
{

}

void FollowLaneManeuver::perform()
{
  
  // get lane marking obj from world worldModel
  LineDetector::Ptr laneResult = _worldModel->getCurrentWorldState()->getCurrentLineDetector();
 
  LineDetection rightDetection = laneResult->rightLine;

  if(!rightDetection.valid){
    _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(0);
    return;
  }
  
  Eigen::Vector3d position = rightDetection.startK;
  Eigen::Vector3d direction = rightDetection.endK - rightDetection.startK;
  Eigen::Vector3d directionNorm = Eigen::Vector3d(direction);
  directionNorm.normalize();


  std::cout << "position x: " << position(0) << " y: " << position(1) << " z: " << position(2) << std::endl;
  std::cout << "direction x: " << direction(0) << " y: " << direction(1) << " z: " << direction(2) << std::endl;
  std::cout << "directionNorm x: " << directionNorm(0) << " y: " << directionNorm(1) << " z: " << directionNorm(2) << std::endl;

  Eigen::Vector3d perpendicular(-directionNorm(1) ,directionNorm(0) ,directionNorm(2));
  Eigen::Vector3d point1 = position + (laneHalfWidth * perpendicular); 
  Eigen::Vector3d point2 = point1 + direction;
  
  
  std::cout << "perpendicular x: " << perpendicular(0) << " y: " << perpendicular(1) << " z: " << perpendicular(2) << std::endl;
  std::cout << "point1 x: " << point1(0) << " y: " << point1(1) << " z: " << point1(2) << std::endl;
  std::cout << "point2 x: " << point2(0) << " y: " << point2(1) << " z: " << point2(2) << std::endl;

//   Eigen::Vector3d carDirection(0, 1, 0);
  
  
  double angle = atan2(point1(1), point1(0)) - atan2(1, 0);
//   double angle = atan2(directionNorm.cross(carDirection), directionNorm.dot(carDirection));
  
  double angleDegrees = angle* 180.0 / M_PI;

  
  std::cout << "FollowLaneManeuver: new steering angle (degrees): " << angleDegrees << std::endl;
  
  _carModel->getServoDrive(AADC_Car::STEERING_SERVO)->setPosition(-angleDegrees);
  _carModel->getMotor(AADC_Car::MAIN_MOTOR)->setSpeed(25);

  
}
