#pragma once

#include "perception/IPerception.h"
#include "ISignDetection.h"
#include "impl/WorldState.h"
#include "IObjectDetection.h"
#include "utils/geometry/Pose2D.h"
#include "utils/geometry/Path2D.h"
#include "impl/DriveInstructionManager.h"
#include "IJuryConstants.h"
#include "IThisCar.h"

#include <boost/smart_ptr.hpp>
#include <vector>

namespace A2O {

/**
 * General Interface for the world model component.
 * The IWorldModel represents all information about the environment.
 *
 * \author Stefan Glaser
 */
class IWorldModel {
public:
  typedef boost::shared_ptr<IWorldModel> Ptr;
  typedef boost::shared_ptr<const IWorldModel> ConstPtr;

  virtual ~IWorldModel(){};
  
  virtual IThisCar::ConstPtr getThisCar() const = 0;
  virtual IThisCar::Ptr getThisCar() = 0;
  
   /** Called to update the world model during an action cycle.
   *
   * \returns success
   */
  virtual bool update(const A2O::IPerception::ConstPtr perception) = 0;
  virtual bool isInitialized() const = 0;
  virtual JuryAction getJuryAction() const = 0;

  virtual WorldState::ConstPtr getCurrentWorldState() const = 0;
  virtual void setNewWorldState(WorldState::ConstPtr newState) = 0;
  
  virtual const Eigen::Vector3d& getFloorNormal() const = 0;

  virtual DriveInstructionManager::Ptr getDriveInstructionManager() = 0;
  
  virtual void stop() = 0;
  virtual void start() = 0;
  
  virtual void setVirtualWayPoint(Eigen::Vector2d point) = 0;
  virtual Eigen::Vector2d getVirtualWayPoint() = 0;
  
  virtual void setVirtualTrail(std::vector<Eigen::Vector2d> trail) = 0;
  virtual std::vector<Eigen::Vector2d> getVirtualTrail() = 0;
  virtual std::vector<RoadSign::Ptr> getRoadSigns() const = 0;
  virtual std::vector<IVisibleObject::Ptr> getObjects() = 0;

  virtual void setAudiRings() = 0;
};
  
}
