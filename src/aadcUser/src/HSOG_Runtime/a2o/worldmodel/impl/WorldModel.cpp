#include "WorldModel.h"
#include "AADCCar.h"
#include "DriveInstructionManager.h"
#include "WayPointExtractor.h"
#include "worldmodel/IJuryConstants.h"
#include <worldmodel/IDriveInstructionConstants.h>
#include <utils/geometry/Geometry.h>
#include "utils/geometry/Pose3D.h"
#include <opencv2/core/core.hpp>

#include <cmath>
#include <iostream>

using namespace std;
using namespace A2O;
using namespace Eigen;


WorldModel::WorldModel(ICarModel::Ptr carModel,
		       ILaneDetection::Ptr laneDetection,
		       ISignDetection::Ptr signDetection,
		       IObjectDetection::Ptr objectDetection,
		       const bool& staticInit)
	: _carModel(carModel),
	  _laneDetection(laneDetection),
	  _signDetection(signDetection),
	  _objectDetection(objectDetection),
	  _gyroOdometry(_carModel->getRearAxle()->getLeftWheel()->getDiameter(), 60),
	  _maneuverManager(boost::make_shared<DriveInstructionManager>("/home/aadc/Maneuverlist.xml")),
	  _thisCar(boost::make_shared<ThisCar>("ThisCar", Pose2D())),
	  _floorDetection(30.0),
	  _initialized(staticInit)
{
  _currentWorldState = boost::make_shared<WorldState>();
  _carPoseOffset = Pose2D();
 
  if (staticInit) {
    cout << "=====================================================================" << endl;
    cout << "================= !! STATIC WORLD INITIALIZATION !! =================" << endl;
    cout << "=====================================================================" << endl;
    
    vector<string> driveInstructions;
    driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_RIGHT);
    driveInstructions.push_back(DRIVE_INSTRUCTION::DRIVE_LEFT);
    _maneuverManager = boost::make_shared<DriveInstructionManager>(driveInstructions);

    Segment::Ptr seg = Segment::createInitialSegment(Pose2D(0, 0, Angle::deg(0)), 8);
    Segment::Ptr newSeg = Segment::appendXCrossing(seg->getStraightOption());
    newSeg = Segment::appendSegment(newSeg->getRightOption(), 1.5);
    newSeg = Segment::appendTCrossingLR(newSeg->getStraightOption());
    // Update driving plan
    seg->update(_maneuverManager, _maneuverManager->getCurrentInstructionIndex());
    
    // Extract new virtual path
    _thisCar->update(Pose2D(), WayPointExtractor::extractPath(seg));
    
    setNewWorldState(_currentWorldState->setCurrentSegment(seg));
    
    _virtualWayPoint = _thisCar->getPath()->interpolateWayPose(_thisCar->getPathPosition() + 18.5).getPosition();
  }

}

WorldModel::~WorldModel()
{

}

IThisCar::ConstPtr WorldModel::getThisCar() const
{
  return boost::dynamic_pointer_cast<const IThisCar>(_thisCar);
}

IThisCar::Ptr WorldModel::getThisCar()
{
  return boost::dynamic_pointer_cast<IThisCar>(_thisCar);
}

bool WorldModel::update(IPerception::ConstPtr perception)
{
    IPointCloudSensor::ConstPtr pointCloudSensor = _carModel->getPointCloudSensor(AADC_Car::FRONT_DEPTH_CAMERA_1);
   _floorNormal = _floorDetection.calculateFloorNormal(pointCloudSensor);
	
   cv::Mat frame = _carModel->getCamera(AADC_Car::FRONT_CAMERA_1)->getImage();

  _signDetection->setFrame(frame);
  _signDetection->notifyPossibleNewFrame();

  // TODO: check what happens if we do not start from a parking spot
  _laneDetection->setFrame(frame, !_initialized && _maneuverManager->getCurrentInstructionIndex() == 0);
  _laneDetection->notifyPossibleNewFrame();

  std::vector<IDistanceSensor::ConstPtr> sensorsIR;
  std::vector<IDistanceSensor::ConstPtr> sensorsUS;
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_FRONT_LEFT));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_FRONT_CENTER_LEFT));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_FRONT_CENTER));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_FRONT_CENTER_RIGHT));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_FRONT_RIGHT));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_SIDE_LEFT));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_SIDE_RIGHT));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_REAR_LEFT));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_REAR_CENTER));
  sensorsUS.push_back(_carModel->getDistanceSensor(AADC_Car::US_REAR_RIGHT));

  // CAUTION: use of car offset pose leads to problems!! If care pose is shifted all object will be shifted, too.
  _objectDetection->setData(_carPoseOffset * getThisCar()->getPose(), pointCloudSensor, sensorsIR, sensorsUS);
  //
  _objectDetection->update();
  
  std::vector<IVisibleObject::Ptr> objectResult = _objectDetection->getObjectResult();
  updateObjectResult(objectResult);

  SignResult::Ptr signResult = _signDetection->getSignResult();
  updateSignResult(signResult);

  LineDetector::Ptr lineResult = _laneDetection->getLaneResult();
  if (lineResult) {
	  lineDetectorToCar(lineResult);
	  processLaneResult(lineResult);
  }
  
  WorldState::ConstPtr newState = getCurrentWorldState()->addLineDetector(lineResult);
  setNewWorldState(newState);

  // get commands from jury module
  IJuryPerceptor::ConstPtr jury = perception->getJuryPerceptor(AADC_Car::JURY_COMMAND);
  if (jury) {
    _maneuverManager->setStartInstructionIndex(jury->getManeuverId());
    _juryAction = jury->getAction(); 
    if(_juryAction == JuryAction::START)
    {
    	_initialized = false;
    }
  }
  
  if (!_initialized) {
    if (_carModel->isInitialized()) {
      ITachometer::ConstPtr leftTacho = _carModel->getTachometer(AADC_Car::LEFT_WHEEL_SPEED);
      ITachometer::ConstPtr rightTacho = _carModel->getTachometer(AADC_Car::RIGHT_WHEEL_SPEED);
      IGyroscope::Ptr gyro = _carModel->getGyro(AADC_Car::CAR_GYRO);
      
      _gyroOdometry.init((leftTacho->getRPM() + rightTacho->getRPM()) * 0.5, gyro->getHorizontalAngle());
      _thisCar->setPose(_gyroOdometry.getPose());

      /*InitialAlignment iniali;
      iniali.parallelParking = false;
      iniali.valid = true;
      iniali.startposition = Vector2d(1, 0.5);
      iniali.endposition = Vector2d(1, -0.5);
      _initialized = createInitialSegment(iniali);*/

      if (lineResult)
      {
	      if (_maneuverManager->getCurrentInstructionIndex() == 0) {
		    _initialized = createInitialSegment(lineResult->initialAlignment);
	      } else {
		    _initialized = createInitialSegment(lineResult);
	      }
      }
    }
    
    lineDetectorToWorld(lineResult);
    return _initialized;
  }

  // Localize based on odometry
  ITachometer::ConstPtr leftTacho = _carModel->getTachometer(AADC_Car::LEFT_WHEEL_SPEED);
  ITachometer::ConstPtr rightTacho = _carModel->getTachometer(AADC_Car::RIGHT_WHEEL_SPEED);
  _thisCar->setPose(_gyroOdometry.update((leftTacho->getRPM() + rightTacho->getRPM()) * 0.5, _carModel->getGyro(AADC_Car::CAR_GYRO)->getHorizontalAngle()));
 
  setSegment();
    
  lineDetectorToWorld(lineResult);
  
  return true;
}

bool WorldModel::isInitialized() const
{
  return _initialized;
}

JuryAction WorldModel::getJuryAction() const
{
  return _juryAction;
}

const Vector3d& WorldModel::getFloorNormal() const
{
  return _floorNormal;
}

DriveInstructionManager::Ptr WorldModel::getDriveInstructionManager()
{
  return _maneuverManager;
}

void WorldModel::stop()
{
  	_laneDetection->stop();
	_signDetection->stop();
	_objectDetection->stop();
}

void WorldModel::start()
{
	_laneDetection->start();
	_signDetection->start();
	_objectDetection->start();
}

void WorldModel::setNewWorldState(WorldState::ConstPtr newState)
{
	lock_guard<mutex> lock(_changeWorldState);

	if (!_currentWorldState || _currentWorldState->isDirectAncestor(newState)) {
		_currentWorldState = newState;
		newState->print();
	} else {
		// someone updated the current world with a version that we are not based on, which means
		// it could contain stuff we don't have --> try to merge it
		_currentWorldState = _currentWorldState->mergeWorldState(_pastWorldStates, newState);
	}

	_pastWorldStates.push_back(_currentWorldState);
}

WorldState::ConstPtr WorldModel::getCurrentWorldState() const
{
	return _currentWorldState;
}

// Segments

void WorldModel::setSegment()
{
  Segment::Ptr currentSegment = _currentWorldState->getCurrentSegment();

  if (!currentSegment.get()) {
    return;
  }
  
  SegmentLink::Ptr intendedLink = currentSegment->getIntendedOption();
  
  if (intendedLink->hasPassedLink(_thisCar->getPose())) {
    // We left the current segment and are now on the updated segment
    Segment::Ptr updatedSegment = intendedLink->getSegmentAfter();
    // Progress driving instruction if necessary
    if (currentSegment->consumesDriveInstruction()) {
      _maneuverManager->progressInstructionIndex();
    }
    
    _segmentDetector.reset();
    updateSegments(updatedSegment);

    // Set new world state
    setNewWorldState(_currentWorldState->setCurrentSegment(updatedSegment)); 
  }
}

void WorldModel::updateSegments(Segment::Ptr segment)
{
    // Update driving plan
    segment->update(_maneuverManager, _maneuverManager->getCurrentInstructionIndex());
    // Extract new virtual path
    _thisCar->setPath(WayPointExtractor::extractPath(segment));
}

bool WorldModel::createInitialSegment(InitialAlignment result)
{
  if (result.valid) {
    // Create initial T-crossing-segment
    Segment::Ptr currentSegment = createInitialSegment(!result.parallelParking, result.startposition, result.endposition);
    
    // Update driving plan
    currentSegment->update(_maneuverManager, _maneuverManager->getCurrentInstructionIndex());
    
    // Extract new virtual path
    _thisCar->setPath(WayPointExtractor::extractPath(currentSegment));
    
    setNewWorldState(_currentWorldState->setCurrentSegment(currentSegment));
    return true;
  }
  
  return false;
}

Segment::Ptr WorldModel::createInitialSegment(const bool& crossParking,
					      const Eigen::Vector2d& p1,
					      const Eigen::Vector2d& p2)
{
  Angle a1 = Angle::to(p1);
  Angle a2 = Angle::to(p2);
  
  Vector2d left = p1;
  Vector2d right = p2;

  if (a1.rad() < a2.rad()) {
    left = p2;
    right = p1;
  }

  Vector2d diffVec = right - left;
  Angle diffAngle = Angle::to(diffVec) + Angle::Deg_90();

  if (crossParking) {
    double x = (diffAngle * left)(0) - 1;

    // Create cross parking initial segment at the calculated pose
    Pose2D initialPose(x, 0.15, diffAngle);
    return Segment::createInitialTCrossingLR(initialPose, Pose2D(-0.15, -0.15));
  } else {
    double y = (diffAngle * left)(1);
    
    // Create parallel parking initial segment at the calculated pose
    Pose2D initialPose(0.15, y, diffAngle);
    return Segment::createInitialTCrossingLR(initialPose, Pose2D(0.0725, -0.25, Angle::deg(-45)));
  }
}

void WorldModel::lineDetectorToCar(LineDetector::Ptr detector)
{
	Pose2D thisPose = _thisCar->getPose();
	Pose3D pos(0, 0, 0);//thisPose.x(), thisPose.y(), 0.0);
	if(detector)
	{
    		ICamera::ConstPtr cam = _carModel->getCamera(AADC_Car::FRONT_CAMERA_1);
		detector->rightLine.startK = pos * cam->toCarSystem(detector->rightLine.startK);
		detector->rightLine.endK = pos * cam->toCarSystem(detector->rightLine.endK);

		detector->stopLine.startK = pos * cam->toCarSystem(detector->stopLine.startK);
		detector->stopLine.endK = pos * cam->toCarSystem(detector->stopLine.endK);
		
		detector->stopLineMiddle.startK = pos * cam->toCarSystem(detector->stopLineMiddle.startK);
		detector->stopLineMiddle.endK = pos * cam->toCarSystem(detector->stopLineMiddle.endK);
		
		detector->startRight.startK = pos * cam->toCarSystem(detector->startRight.startK);
		detector->startRight.endK = pos * cam->toCarSystem(detector->startRight.endK);
		
		detector->endRight.startK = pos * cam->toCarSystem(detector->endRight.startK);
		detector->endRight.endK = pos * cam->toCarSystem(detector->endRight.endK);
		
		detector->stopRight.startK = pos * cam->toCarSystem(detector->stopRight.startK);
		detector->stopRight.endK = pos * cam->toCarSystem(detector->stopRight.endK);
		
		detector->stopRightMiddle.startK = pos * cam->toCarSystem(detector->stopRightMiddle.startK);
		detector->stopRightMiddle.endK = pos * cam->toCarSystem(detector->stopRightMiddle.endK);
		
		detector->leftLine.startK = pos * cam->toCarSystem(detector->leftLine.startK);
		detector->leftLine.endK = pos * cam->toCarSystem(detector->leftLine.endK);
		
		detector->endLeft.startK = pos * cam->toCarSystem(detector->endLeft.startK);
		detector->endLeft.endK = pos * cam->toCarSystem(detector->endLeft.endK);
		
		detector->upperRight.startK = pos * cam->toCarSystem(detector->upperRight.startK);
		detector->upperRight.endK = pos * cam->toCarSystem(detector->upperRight.endK);
		
		detector->upperRightStart.startK = pos * cam->toCarSystem(detector->upperRightStart.startK);
		detector->upperRightStart.endK = pos * cam->toCarSystem(detector->upperRightStart.endK);
		
		detector->upperLeft.startK = pos * cam->toCarSystem(detector->upperLeft.startK);
		detector->upperLeft.endK = pos * cam->toCarSystem(detector->upperLeft.endK);
		
		detector->upperLeftStart.startK = pos * cam->toCarSystem(detector->upperLeftStart.startK);
		detector->upperLeftStart.endK = pos * cam->toCarSystem(detector->upperLeftStart.endK);
		
		detector->leftCurve.startK = pos * cam->toCarSystem(detector->leftCurve.startK);
		detector->leftCurve.middleK = pos * cam->toCarSystem(detector->leftCurve.middleK);
		detector->leftCurve.endK = pos * cam->toCarSystem(detector->leftCurve.endK);
		
		detector->rightCurve.startK = pos * cam->toCarSystem(detector->rightCurve.startK);
		detector->rightCurve.middleK = pos * cam->toCarSystem(detector->rightCurve.middleK);
		detector->rightCurve.endK = pos * cam->toCarSystem(detector->rightCurve.endK);
	}
}

void WorldModel::lineDetectorToWorld(LineDetector::Ptr detector)
{
	Pose2D thisPose = _thisCar->getPose();
	Pose3D pos(thisPose.x(), thisPose.y(), 0.0, AngleAxisd(thisPose.getAngle().rad(), Vector3d(0, 0, 1)));
	if(detector)
	{
		detector->rightLine.startK = pos * detector->rightLine.startK;
		detector->rightLine.endK = pos * detector->rightLine.endK;

		detector->stopLine.startK = pos * detector->stopLine.startK;
		detector->stopLine.endK = pos * detector->stopLine.endK;
		
		detector->stopLineMiddle.startK = pos * detector->stopLineMiddle.startK;
		detector->stopLineMiddle.endK = pos * detector->stopLineMiddle.endK;
		
		detector->startRight.startK = pos * detector->startRight.startK;
		detector->startRight.endK = pos * detector->startRight.endK;
		
		detector->endRight.startK = pos * detector->endRight.startK;
		detector->endRight.endK = pos * detector->endRight.endK;
		
		detector->stopRight.startK = pos * detector->stopRight.startK;
		detector->stopRight.endK = pos * detector->stopRight.endK;
		
		detector->stopRightMiddle.startK = pos * detector->stopRightMiddle.startK;
		detector->stopRightMiddle.endK = pos * detector->stopRightMiddle.endK;
		
		detector->leftLine.startK = pos * detector->leftLine.startK;
		detector->leftLine.endK = pos * detector->leftLine.endK;
		
		detector->endLeft.startK = pos * detector->endLeft.startK;
		detector->endLeft.endK = pos * detector->endLeft.endK;
		
		detector->upperRight.startK = pos * detector->upperRight.startK;
		detector->upperRight.endK = pos * detector->upperRight.endK;
		
		detector->upperRightStart.startK = pos * detector->upperRightStart.startK;
		detector->upperRightStart.endK = pos * detector->upperRightStart.endK;
		
		detector->upperLeft.startK = pos * detector->upperLeft.startK;
		detector->upperLeft.endK = pos * detector->upperLeft.endK;
		
		detector->upperLeftStart.startK = pos * detector->upperLeftStart.startK;
		detector->upperLeftStart.endK = pos * detector->upperLeftStart.endK;
		
		detector->leftCurve.startK = pos * detector->leftCurve.startK;
		detector->leftCurve.middleK = pos * detector->leftCurve.middleK;
		detector->leftCurve.endK = pos * detector->leftCurve.endK;
		
		detector->rightCurve.startK = pos * detector->rightCurve.startK;
		detector->rightCurve.middleK = pos * detector->rightCurve.middleK;
		detector->rightCurve.endK = pos * detector->rightCurve.endK;
	}
}

bool WorldModel::createInitialSegment(LineDetector::Ptr detector)
{
	if(detector && detector->rightLine.valid && detector->rightLine.getLength() > 0.5){
		Vector2d pos(detector->rightLine.startK(0), detector->rightLine.startK(1));
		Vector2d dir(detector->rightLine.endK(0) - detector->rightLine.startK(0),
				detector->rightLine.endK(1) - detector->rightLine.startK(1));

		Angle a1 = Angle::to(dir(0), dir(1));
		
		Vector2d localPos = a1 * pos;

		Pose2D startPose = Pose2D(-0.1, localPos(1) + 0.45, a1);
		cout << "startPose: " << startPose.x() << " " << startPose.y() << " " << startPose.getAngle().deg() << endl;
		Segment::Ptr currentSegment = Segment::createInitialSegment(startPose, 10.0);
		
		// Update driving plan
		currentSegment->update(_maneuverManager, _maneuverManager->getCurrentInstructionIndex());
		
		// Extract new virtual path
		_thisCar->setPath(WayPointExtractor::extractPath(currentSegment));
		
		setNewWorldState(_currentWorldState->setCurrentSegment(currentSegment));
		return true;
	  }
	return false;
}

void WorldModel::processLaneResult(LineDetector::Ptr detector)
{
  if (!detector) {
    return;
  }

  Segment::Ptr currentSegment = _currentWorldState->getCurrentSegment();
  if (!currentSegment) {
    return;
  }
  
  Pose2D thisPose = _thisCar->getPose();

  if (currentSegment->isCrossing()) {
      // for now we do not consider nextSegment when inside crossing
      // at least when we are turning we should keep this
      // so we might add code in case of going straight

  } else {
    switch (detector->nextSegment) {
      case BCLEFT:
      case SCLEFT:
	if (detector->rightCurve.valid) {
	  Circle2D circle = thisPose * Circle2D(detector->rightCurve.centerK(0), detector->rightCurve.centerK(1), detector->rightCurve.radius);
	  _segmentDetector.addCurveDetection(currentSegment, circle, true);
	}
	break;
      case BCRIGHT:
      case SCRIGHT:
	if (detector->leftCurve.valid) {
	  Circle2D circle = thisPose * Circle2D(detector->leftCurve.centerK(0), detector->leftCurve.centerK(1), detector->leftCurve.radius);
	  _segmentDetector.addCurveDetection(currentSegment, circle, false);
	}
	break;
      case STRAIGHT:
	_segmentDetector.addStraightDetection(currentSegment, thisPose * detector->nextSegmentPose);
	break;
      case TLEFT:
	_segmentDetector.addTCrossingLSDetection(currentSegment, thisPose * detector->nextSegmentPose);
	break;
      case TRIGHT:
	_segmentDetector.addTCrossingSRDetection(currentSegment, thisPose * detector->nextSegmentPose);
	break;
      case TLEFTRIGHT:
	_segmentDetector.addTCrossingLRDetection(currentSegment, thisPose * detector->nextSegmentPose);
	break;
      case XCROSSROAD:
	_segmentDetector.addXCrossingDetection(currentSegment, thisPose * detector->nextSegmentPose);
	break;
      default:
	break;
    }
    
    if (currentSegment->hasIntendedOption()
	&& currentSegment->getIntendedOption()->hasSegmentAfter()
	&& currentSegment->getIntendedOption()->getSegmentAfter()->isCrossing()) {
      Pose2D relativeCrossingPose = currentSegment->getIntendedOption()->getPose() / thisPose;
      if (relativeCrossingPose.x() < -0.8) {
	_segmentDetector.redefineNextSegment(currentSegment);
      }
    } else {
      _segmentDetector.redefineNextSegment(currentSegment);
    }
    
    updateSegments(currentSegment);
  }
  
  resetOdometry(detector);
}


void WorldModel::resetOdometry(LineDetector::Ptr detector)
{
  Segment::Ptr currentSegment = _currentWorldState->getCurrentSegment();
  Pose2D thisPose = _thisCar->getPose();
  
  if (!currentSegment) {
    return;
  }
  
   // Reset odometry
  if (currentSegment->isStraight() && !currentSegment->isCrossing()) {
    /*if (false && detector->rightLine.valid && detector->rightLine.getLength() > 0.8
	  && detector->leftLine.valid && detector->leftLine.getLength() > 0.8) {
      Vector3d rightDir = detector->rightLine.endK - detector->rightLine.startK;
      // Vector3d leftDir = newResult->leftLane.direction;
      Vector3d rightLanePos = detector->rightLine.startK;
      Vector3d leftLanePos = detector->leftLine.startK;
      Pose2D beginPose = currentSegment->getBeginLink()->getPose();
      Pose2D relativePose = beginPose / thisPose;
      Pose2D newRelativePose = Pose2D(relativePose.x(),
    		  (relativePose.y() - rightLanePos(1) - leftLanePos(1)) / 3, Angle::to(rightDir(0), rightDir(1)).negate());
      Pose2D newCarPose = beginPose * newRelativePose;
      _gyroOdometry.reset(newCarPose);
      _thisCar->setPose(newCarPose);
      _carPoseOffset = _carPoseOffset + (thisPose / newCarPose);
    } else if (detector->rightLine.valid && detector->rightLine.getLength() > 0.8) {
      Vector3d dir = detector->rightLine.endK - detector->rightLine.startK;
      Vector3d rightLanePos = detector->rightLine.startK;
      Pose2D beginPose = currentSegment->getBeginLink()->getPose();
      Pose2D relativePose = beginPose / thisPose;
      Pose2D newRelativePose = Pose2D(relativePose.x(),
    		  (relativePose.y() - rightLanePos(1) - 0.45) / 2, Angle::to(dir(0), dir(1)).negate());
      Pose2D newCarPose = beginPose * newRelativePose;
      _gyroOdometry.reset(newCarPose);
      _thisCar->setPose(newCarPose);
      _carPoseOffset = _carPoseOffset + (thisPose / newCarPose);
    } else if (false && detector->leftLine.valid && detector->leftLine.getLength() > 0.8) {
      Vector3d dir = detector->leftLine.endK - detector->leftLine.startK;
      Vector3d leftLanePos = detector->leftLine.startK;
      Pose2D beginPose = currentSegment->getBeginLink()->getPose();
      Pose2D relativePose = beginPose / thisPose;
      Pose2D newRelativePose = Pose2D(relativePose.x(),
    		  (relativePose.y() - leftLanePos(1) + 0.45) / 2, Angle::to(dir(0), dir(1)).negate());
      Pose2D newCarPose = beginPose * newRelativePose;
      _gyroOdometry.reset(newCarPose);
      _thisCar->setPose(newCarPose);
      _carPoseOffset = _carPoseOffset + (thisPose / newCarPose);
    }*/


    
    if (detector->rightLine.valid && detector->rightLine.getLength() > 0.4) {
      Vector3d diffVec3d = detector->rightLine.endK - detector->rightLine.startK;
      Vector2d diffVec(diffVec3d(0), diffVec3d(1));
      Pose2D carLocalLinePose(detector->rightLine.startK(0), detector->rightLine.startK(1), Angle::to(diffVec));
      Pose2D beginPose = currentSegment->getBeginLink()->getPose();
      Pose2D segmentLocalCarPose = beginPose / thisPose;
      Pose2D segmentLocalLinePose = segmentLocalCarPose * carLocalLinePose;
      Pose2D shiftPose(0, -0.45 - segmentLocalLinePose.y(), segmentLocalLinePose.getAngle().negate());
      Pose2D newCarPose = thisPose * shiftPose;
      _gyroOdometry.reset(newCarPose);
      _thisCar->setPose(newCarPose);
      _carPoseOffset = _carPoseOffset + shiftPose;
    }
    
    if (false && detector->stopLine.valid
	&& currentSegment->hasIntendedOption()
	&& currentSegment->getIntendedOption()->hasSegmentAfter()
	&& currentSegment->getIntendedOption()->getSegmentAfter()->isCrossing()) {
      Vector3d diffVec = detector->stopLine.endK - detector->stopLine.startK;
      Pose2D linePose(detector->stopLine.startK(0), detector->stopLine.startK(1), Angle::to(diffVec(0), diffVec(1)));
      double distance = linePose.invert().y() + 0.1;
      
      Pose2D nextLinkPose = currentSegment->getIntendedOption()->getPose();
      Pose2D localCarPose = nextLinkPose / thisPose;
      Pose2D newLocalCarPose(distance, localCarPose.y(), localCarPose.getAngle());
      Pose2D newCarPose = nextLinkPose * newLocalCarPose;
      _gyroOdometry.reset(newCarPose);
      _thisCar->setPose(newCarPose);
      _carPoseOffset = _carPoseOffset + (thisPose / newCarPose);
    }
  }
}


void WorldModel::updateObjectResult(vector< IVisibleObject::Ptr > visibleObjects)
{
  if(visibleObjects.size() > 0)
  {
    _visibleObjects = visibleObjects;
  }
}

vector< IVisibleObject::Ptr > WorldModel::getObjects()
{
  return _visibleObjects;
}


void WorldModel::updateSignResult(SignResult::Ptr newResult){
	if (!newResult) {
		return;
	}
	Segment::Ptr segment = _currentWorldState->getCurrentSegment();
	if (!segment) {
		return;
	}
	vector<RoadSign> signs = newResult->getSigns();
	
	for(unsigned int i=0; i < signs.size(); i++)
	{	
		int index = addToSeenSigns(signs[i]);
		if(index != -1 && _seenRoadSigns[index]->getSeenCount()  > 3) {
			cout << "seen roadsign" << index << endl;
			addSignToCurrentSegments(_seenRoadSigns[index]);

		}
	}	
	WorldState::ConstPtr newState = getCurrentWorldState()->addSignResult(*newResult);
	setNewWorldState(newState);
}

int WorldModel::addToSeenSigns(RoadSign toAdd)
{
	// to global
	toAdd.setPose(_thisCar->getPose() * toAdd.getPose());

	int timesSeen = 0;
	double angle = toAdd.getPose().getAngle().deg();
	if(angle < 80 || angle > 120)
		return -1;
	// try to match to an already existing sign
	unsigned int j = 0;
	for(; j < _seenRoadSigns.size(); j++) {
		auto existingSign = _seenRoadSigns[j];
		if(existingSign->getSignType() == toAdd.getSignType()) {
			Pose2D toAddPose = toAdd.getPose();
			Pose2D existingPose = existingSign->getPose();

			if(existingPose.getDistanceTo(toAddPose) < 0.2) {
				existingSign->update(toAddPose);
				timesSeen = existingSign->getSeenCount();
				break;
			}
		}
	}
	if (!timesSeen)
	{
		_seenRoadSigns.push_back(boost::make_shared<RoadSign>(toAdd));
		return _seenRoadSigns.size()-1;
	}
	return j;
}

void WorldModel::addSignToCurrentSegments(RoadSign::Ptr sign)
{
        // for the first segment we have to consider our pose
        // for the next, it only can be the straight
	Segment::Ptr segment = _currentWorldState->getCurrentSegment();
	if(!segment) {
		return;
	}
	SegmentLink::Ptr link = segment->getClosestLinkToPose(_thisCar->getPose());

	vector<SegmentLink::Ptr> links;
	links.push_back(link);

	if(link->hasSegmentAfter()) {
		segment = link->getSegmentAfter();
	}

	while(segment && segment->hasStraightOption()) {
		if(segment->hasStraightOption()){
			link = segment->getStraightOption();
		}
		// for t left right
		if(segment->hasLeftOption()){
			links.push_back(segment->getLeftOption());
		}
		if(segment->hasRightOption()){
			links.push_back(segment->getRightOption());
		}
		links.push_back(link);
		segment = link->getSegmentAfter();
	}

	// find closest link
	Pose2D signPose = sign->getPose();
	
	double minDistance = numeric_limits<double>::max();
	int index = -1;
	for(unsigned int i = 0; i < links.size(); i++)
	{
		double distance = signPose.getDistanceTo(links[i]->getPose());
		if(distance > 0 && distance < 1.5 && distance < minDistance) {
			minDistance = distance;
			index = i;
		}
	}
	if(index != -1) {
		vector<RoadSign::Ptr> roadSigns;
		roadSigns.push_back(sign);
		links[index]->getSegmentBefore()->attachRoadSigns(roadSigns);
	}
}

void WorldModel::addSignsToNewSegment(Segment::Ptr newSegment)
{
	Pose2D segmentPose = newSegment->getBeginLink()->getPose();

	int counter = 0;
	// check angle??
	for(unsigned int i = 0; i < _seenRoadSigns.size(); i++)
	{
		double distance = _seenRoadSigns[i]->getPose().getDistanceTo(segmentPose);
		if(distance > 0 && distance < 1.5 && _seenRoadSigns[i]->getSeenCount() > 3) {
			vector<RoadSign::Ptr> roadSigns;
			roadSigns.push_back(_seenRoadSigns[i]);
			newSegment->attachRoadSigns(roadSigns);
			counter++;
		}
	}
	if(counter > 1) {
		std::cout << "WARNING: got more then one matching RoadSign for new Segment" << std::endl;
	}
}

vector<RoadSign::Ptr> WorldModel::getRoadSigns() const
{
	return _seenRoadSigns;
}

Vector2d WorldModel::getVirtualWayPoint()
{
  return _virtualWayPoint;
}

void WorldModel::setVirtualWayPoint(Vector2d point)
{
  _virtualWayPoint = point;
}

vector< Vector2d > WorldModel::getVirtualTrail()
{
  return _virtualTrail;
}

void WorldModel::setVirtualTrail(vector< Vector2d > trail)
{
  _virtualTrail = trail;
}

void WorldModel::setAudiRings()
{
  double length = M_PI;
  
  // Ring 1
  Segment::Ptr seg = Segment::createInitialSegment(Pose2D(), length, Angle::rad(M_PI));
  Segment::Ptr newSeg = seg;
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(M_PI));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(M_PI));
  
  // Ring 2
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(-M_PI));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(-M_PI));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(-M_PI));
  
  // Ring 3
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(M_PI));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(M_PI));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(M_PI));
  
  // Ring 4
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(-M_PI));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(-M_PI));
  newSeg = Segment::appendSegment(newSeg->getStraightOption(), length, Angle::rad(-M_PI));
  
  seg->update(_maneuverManager, _maneuverManager->getCurrentInstructionIndex());
    
  // Extract new virtual path
  _gyroOdometry.reset(Pose2D());
  _thisCar->setPose(Pose2D());
  _thisCar->setPath(WayPointExtractor::extractPath(seg));
    
  setNewWorldState(_currentWorldState->setCurrentSegment(seg));
}
