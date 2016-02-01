#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <deque>

#include "VersionIdGenerator.h"
#include "worldmodel/signdetection/SignResult.h"
#include "worldmodel/lanedetection/LineDetector.h"
#include "worldmodel/IVisibleObject.h"
#include "worldmodel/street/Segment.h"


#include "utils/persistence/PersistentVector.h"

namespace A2O {

	/**
	 * The WorldState represents all information at a certain time about the environment of the car.
	 * The class is immutable, so a state can be used in several threads without interference.
	 */

	class WorldState{

		// needs to be able to read private members 
		class WorldStateBuilder;
		friend WorldState::WorldStateBuilder;

		public:

		typedef boost::shared_ptr<WorldState> Ptr;
		typedef boost::shared_ptr<const WorldState> ConstPtr;

		WorldState(WorldState* parent=NULL){
			setupIds(parent);
		};
		WorldState(WorldState* parent, PersistentVector<SignResult> roadSigns,
			   Segment::Ptr currentSegment,
			   LineDetector::Ptr detector
			  );
		virtual ~WorldState();

	
		// for testing of ancestry, etc.
		WorldState::ConstPtr getChild() const;

	
		Segment::Ptr getCurrentSegment() const;
		WorldState::ConstPtr setCurrentSegment(Segment::Ptr) const;

		
		bool isDirectAncestor(WorldState::ConstPtr newWorld) const;

		WorldState::ConstPtr addSignResult(SignResult toAdd) const;
		PersistentVector<SignResult> getRoadSigns() const;

		WorldState::ConstPtr addLineDetector(LineDetector::Ptr result) const;
		const LineDetector::Ptr getCurrentLineDetector() const;

		WorldState::ConstPtr setObjects(std::vector<IVisibleObject::Ptr>  obstacles) const;
		const std::vector<IVisibleObject> getCurrentObjects() const;

				
		int getId() const;
		void print() const;
		WorldState::ConstPtr mergeWorldState(std::deque<WorldState::ConstPtr> _oldStates, WorldState::ConstPtr& newState) const;
		WorldState::ConstPtr findCommonAncestor(std::deque<WorldState::ConstPtr> states, WorldState::ConstPtr newState) const;

	
		
		private:
		
		void setupIds(const WorldState* parent);
		VersionInfo::ConstPtr getVersionInfo() const { return _version; };

		VersionIdGenerator* _generator;
		VersionInfo::ConstPtr _version;

		/* data goes here */
		PersistentVector<SignResult> _roadSigns;

		Segment::Ptr _currentSegment;
		LineDetector::Ptr _detector;

	};


	/**
	 * Used for constructing a WorldState, so we dont need many different constructors
	 */

	class WorldState::WorldStateBuilder : public WorldState{

		private:
			const WorldState* _parent;

		public:
			WorldStateBuilder(const WorldState* parent)
			{
				// needed for the ids
				_parent = parent;
				// copy all of the members of parent
				this->_roadSigns = parent->_roadSigns;
				this->_currentSegment = parent->_currentSegment;
				this->_detector = parent->_detector;
			}
			WorldState::ConstPtr persist();
			void addDetectedRoadSign(SignResult toAdd);
			void addLineDetector(LineDetector::Ptr result);

			void setCurrentSegment(Segment::Ptr segment);

	};

}
