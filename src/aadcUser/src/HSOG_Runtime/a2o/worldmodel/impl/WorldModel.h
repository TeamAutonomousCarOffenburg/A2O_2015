#pragma once

#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>

#include <mutex>
#include <deque>
#include <vector>
#include <utility>

#include "worldmodel/IWorldModel.h"
#include "AADCCar.h"
#include "carmodel/ICarModel.h"
#include "../ILaneDetection.h"
#include "../IObjectDetection.h"
#include "DriveInstructionManager.h"
#include "worldmodel/odometry/GyroOdometry.h"
#include "ThisCar.h"
#include "worldmodel/street/SegmentDetector.h"

#include "worldmodel/street/Segment.h"
#include "worldmodel/street/SegmentLink.h"
#include "worldmodel/lanedetection/InitialAlignment.h"
#include "FloorNormalDetection.h"

#include "WorldState.h"

namespace A2O {

/**
 * The WorldModel represents all information about the environment of the car.
 *
 * \author Stefan Glaser
 */
class WorldModel : public virtual IWorldModel{
public:
  WorldModel(ICarModel::Ptr carModel,
	     ILaneDetection::Ptr laneDetection,
	     ISignDetection::Ptr signDetection,
	     IObjectDetection::Ptr objectDetection,
	     const bool& staticInit = false);
  virtual ~WorldModel();
  
  virtual IThisCar::ConstPtr getThisCar() const;
  virtual IThisCar::Ptr getThisCar();
  
  virtual bool update(IPerception::ConstPtr perception);
  virtual bool isInitialized() const;
  virtual JuryAction getJuryAction() const;
  
  void stop();
  void start();

  virtual void setNewWorldState(WorldState::ConstPtr _newState);
  virtual WorldState::ConstPtr getCurrentWorldState() const;
 
  void processLaneResult(LineDetector::Ptr detector);


  void updateSignResult(SignResult::Ptr newResult);
  std::vector<RoadSign::Ptr> getRoadSigns() const;
  
  void updateObjectResult(std::vector<IVisibleObject::Ptr> visibleObjects);
  virtual std::vector<IVisibleObject::Ptr> getObjects();
  
  void setSegment();
  
  virtual const Eigen::Vector3d& getFloorNormal() const;

  virtual DriveInstructionManager::Ptr getDriveInstructionManager();
  
  virtual void setVirtualWayPoint(Eigen::Vector2d point);
  virtual Eigen::Vector2d getVirtualWayPoint();
  
  virtual void setVirtualTrail(std::vector<Eigen::Vector2d> trail);
  virtual std::vector<Eigen::Vector2d> getVirtualTrail();

  virtual void setAudiRings();

protected:
  void lineDetectorToCar(LineDetector::Ptr detector);
  void lineDetectorToWorld(LineDetector::Ptr detector);

  void updateSegments(Segment::Ptr segment);
  
  bool createInitialSegment(LineDetector::Ptr detector);
  bool createInitialSegment(InitialAlignment result);
  Segment::Ptr createInitialSegment(const bool& crossParking,
				    const Eigen::Vector2d& p1,
				    const Eigen::Vector2d& p2);
  
  
  void resetOdometry(LineDetector::Ptr detector);
  int addToSeenSigns(RoadSign toAdd);
  void addSignToCurrentSegments(RoadSign::Ptr sign);
  void addSignsToNewSegment(Segment::Ptr newSegment);

  /** The car model. */
  ICarModel::Ptr _carModel;
  ILaneDetection::Ptr _laneDetection;
  ISignDetection::Ptr _signDetection;
  IObjectDetection::Ptr _objectDetection;
  GyroOdometry _gyroOdometry;
  DriveInstructionManager::Ptr _maneuverManager;
  
  ThisCar::Ptr _thisCar;
  
  Eigen::Vector3d _floorNormal;
  FloorNormalDetection _floorDetection;
  
  // als übergangslösung letztes objekt
  std::vector<VisibleObject::Ptr> _objects;
  
private:
  Pose2D _carPoseOffset;
  WorldState::ConstPtr _currentWorldState;
  std::mutex _changeWorldState;

  std::deque<WorldState::ConstPtr> _pastWorldStates;

  std::vector<RoadSign::Ptr> _seenRoadSigns;
  bool _initialized;
  
  std::vector<IVisibleObject::Ptr> _visibleObjects;
  
  JuryAction _juryAction;
  
  Eigen::Vector2d _virtualWayPoint;
  std::vector<Eigen::Vector2d> _virtualTrail;
  
  SegmentDetector _segmentDetector;
};
}
