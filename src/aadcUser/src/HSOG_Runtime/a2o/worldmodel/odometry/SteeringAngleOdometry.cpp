#include "SteeringAngleOdometry.h"

#include "utils/geometry/Line2D.h"
#include "utils/geometry/Circle2D.h"
#include "utils/geometry/Geometry.h"


using namespace A2O;
using namespace Eigen;
using namespace std;


SteeringAngleOdometry::SteeringAngleOdometry(const double& wheelDiameter, const Vector3d& frontAxlePosition, const double& frontAxleLength,
					     const double& rearAxleLength)
	  : _wheelDiameter(wheelDiameter) , _frontAxlePosition(frontAxlePosition), 
		_frontAxleHalfLength(frontAxleLength * 0.5), _rearAxleHalfLength(rearAxleLength * 0.5)
{
  _wheelTickOffset = 0;
}

Pose2D SteeringAngleOdometry::update(const double& wheelTicks, const double& steeringAngle)
{
  double newWheelTicks = wheelTicks - _wheelTickOffset;
  // not travelled
  if(std::fabs(newWheelTicks) < 0.001)
    return _pose;
  
  _wheelTickOffset = wheelTicks;
  
  double distanceTravelled = newWheelTicks * (M_PI / 8) * _wheelDiameter * CORRECTION_FACTOR;
  
  // threshold: angle = 0
  if(std::fabs(steeringAngle) < 0.5)
  {
    Vector2d offsetVector =  _pose.getAngle().getVector();
    _pose.setPosition(_pose.getPosition() + offsetVector * distanceTravelled);
    return _pose;
  }
  else 
  {     
     // front circle at circle inner wheel due to AADC Car steering sensor value
     Vector2d innerFrontWheelOffset = Vector2d(_frontAxlePosition(0), _frontAxlePosition(1));
     // if steering angle points to the left, use left wheel
     if(steeringAngle > 0)
       innerFrontWheelOffset(1) += (_frontAxleHalfLength);
     else
       innerFrontWheelOffset(1) -= (_frontAxleHalfLength); 
     
     // set coordinates in world system     
     Vector2d frontInnerWheelPosition = _pose.getAngle() * innerFrontWheelOffset + _pose.getPosition();
     // CAUTION: steering angle: left positiv
     Vector2d normalVectorFront = Angle::deg(_pose.getAngle().deg() + steeringAngle).normal().getVector();
     
     // calculate the factor of the steering influence of the rear wheels
     double effectiveRearAxleSteeringAngle;
     if(fabs(steeringAngle) < REAR_STEERING_LOWER_THRESHOLD)
       effectiveRearAxleSteeringAngle = 0.0;
     else if(fabs(steeringAngle) < REAR_STEERING_UPPER_THRESHOLD)
       effectiveRearAxleSteeringAngle = ANGLE_REAR_WHEELS * (steeringAngle - REAR_STEERING_LOWER_THRESHOLD) 
					  / (REAR_STEERING_UPPER_THRESHOLD - REAR_STEERING_LOWER_THRESHOLD);
     else
       effectiveRearAxleSteeringAngle = ANGLE_REAR_WHEELS;

     // set steering influence of the rear wheels
     Vector2d rearExteriorWheelPosition = _pose.getPosition();
     Vector2d normalVectorRear;
     if(steeringAngle > 0)
     {
       rearExteriorWheelPosition += _pose.getAngle() * Vector2d(0, -_rearAxleHalfLength);
       normalVectorRear = (_pose.getAngle().normal() + Angle::deg(effectiveRearAxleSteeringAngle)).getVector();

     }
     else
     {
       rearExteriorWheelPosition += _pose.getAngle() * Vector2d(0, _rearAxleHalfLength);
       normalVectorRear = (_pose.getAngle().normal() - Angle::deg(effectiveRearAxleSteeringAngle)).getVector();
     }

     // calculate the cirle that is driven by the middle of the rear axle
     Vector2d circleOrigin;
     Line2D l1(frontInnerWheelPosition, frontInnerWheelPosition + normalVectorFront);
     Line2D l2(rearExteriorWheelPosition, rearExteriorWheelPosition + normalVectorRear);
     Geometry::intersectionPoint(l1, l2, circleOrigin);
     Circle2D rearAxleCircle(circleOrigin, _pose.getPosition());
     
     // calculate the part of the circle that is driven and get the new point
     short direction = rearAxleCircle.getDirectionOfRotation(_pose.getPosition(), _pose.getAngle());
     Angle segmentAngle = rearAxleCircle.getAngle(distanceTravelled * direction);
     Angle newPointAngle = rearAxleCircle.getAngle(_pose.getPosition()) + segmentAngle;
     Vector2d newPoint = rearAxleCircle.getPointOnCircle(newPointAngle);
     Angle newPoseAngle = rearAxleCircle.slope(newPoint);
     
     // correct the drift factor of the rear wheels on the pose angle
     if(steeringAngle > 0)
       newPoseAngle -= Angle::deg(effectiveRearAxleSteeringAngle);
     else
       newPoseAngle += Angle::deg(effectiveRearAxleSteeringAngle);
     // if direction of the pose on circle is wrong, change it
     if(std::fabs(direction - rearAxleCircle.getDirectionOfRotation(newPoint, newPoseAngle)) > 0.001)
       newPoseAngle = newPoseAngle.opposite();
         
     // set new pose and return it
     _pose.setPosition(newPoint);
     _pose.setAngle(newPoseAngle);
     return _pose;
  }
}

Pose2D& SteeringAngleOdometry::getCurrentPose()
{
  return _pose;
}

void SteeringAngleOdometry::setCurrentPose(const Pose2D& pose)
{
  _pose = Pose2D(pose);
}





