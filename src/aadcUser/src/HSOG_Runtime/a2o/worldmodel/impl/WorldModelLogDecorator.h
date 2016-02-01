#pragma once

#include <string>

#include "worldmodel/IWorldModel.h"
#include "utils/logger/EventLogger.h"
#include "utils/geometry/Pose2D.h"

namespace A2O {

	class WorldModelLogDecorator: public virtual IWorldModel {
		public:
			WorldModelLogDecorator(IWorldModel::Ptr worldModel, std::string logfile)
			{
				_worldModel = worldModel;
				_logger = EventLogger::getLogger(logfile);
			}
			virtual IThisCar::ConstPtr getThisCar() const
			{
				return _worldModel->getThisCar();
			};
			virtual IThisCar::Ptr getThisCar()
			{
			  return _worldModel->getThisCar();
			};
			virtual void setAudiRings()
			{
				_worldModel->setAudiRings();
			};

			virtual bool update(const A2O::IPerception::ConstPtr perception){
				bool result = _worldModel->update(perception);

				_logger->logPose("CAR", _worldModel->getThisCar()->getPose());
				return result;
			}
			virtual bool isInitialized() const
			{
				return _worldModel->isInitialized();
			};
			virtual JuryAction getJuryAction() const
			{
				return _worldModel->getJuryAction();
			};
			virtual WorldState::ConstPtr getCurrentWorldState() const{
				return	_worldModel->getCurrentWorldState();	
			}
			virtual void setNewWorldState(WorldState::ConstPtr newState){
				_worldModel->setNewWorldState(newState);
			}
			
			virtual void stop(){
				_worldModel->stop();
			}
			virtual void start(){
				_worldModel->start();	
			}

			virtual DriveInstructionManager::Ptr getDriveInstructionManager()
			{
			  return _worldModel->getDriveInstructionManager();
			}
			
			virtual Eigen::Vector2d getVirtualWayPoint()
			{
			  return _worldModel->getVirtualWayPoint();
			}

			virtual void setVirtualWayPoint(Eigen::Vector2d point)
			{
			  _worldModel->setVirtualWayPoint(point);
			}
			
			virtual void setVirtualTrail(std::vector< Eigen::Vector2d > trail)
			{
			  _worldModel->setVirtualTrail(trail);
			}
			
			virtual std::vector< Eigen::Vector2d > getVirtualTrail()
			{
			  return _worldModel->getVirtualTrail();
			}
			virtual const Eigen::Vector3d& getFloorNormal() const
			{
				return _worldModel->getFloorNormal();
			}
  			virtual std::vector<RoadSign::Ptr> getRoadSigns() const
			{
				return _worldModel->getRoadSigns();
			}
			virtual std::vector<IVisibleObject::Ptr> getObjects()
			{
			      return _worldModel->getObjects();
			}
			
		private:

			IWorldModel::Ptr _worldModel;
			EventLogger* _logger;

	};
}
