#include "WorldState.h"
#include <iostream>

using namespace A2O;

WorldState::WorldState(WorldState *parent, PersistentVector<SignResult> roadSigns, Segment::Ptr currentSegment, LineDetector::Ptr detector)
{
	setupIds(parent);
	_roadSigns = roadSigns;
	_currentSegment = currentSegment;
	_detector = detector;
}

WorldState::~WorldState()
{

}

WorldState::ConstPtr WorldState::addSignResult(const SignResult toAdd) const
{

	WorldStateBuilder builder(this);
	builder.addDetectedRoadSign(toAdd);

	return builder.persist(); 
}

WorldState::ConstPtr WorldState::addLineDetector(LineDetector::Ptr result) const
{
	WorldStateBuilder builder(this);
	builder.addLineDetector(result);

	return builder.persist();
}

WorldState::ConstPtr WorldState::getChild() const
{
	WorldStateBuilder builder(this);
	return builder.persist();
}



WorldState::ConstPtr WorldState::setCurrentSegment(Segment::Ptr segment) const
{
  WorldStateBuilder builder(this);
  builder.setCurrentSegment(segment);
  return builder.persist();
}

PersistentVector<SignResult> WorldState::getRoadSigns() const
{
	return _roadSigns;
}

const LineDetector::Ptr WorldState::getCurrentLineDetector() const
{
	return _detector;
}

const std::vector<IVisibleObject> WorldState::getCurrentObjects() const
{
	return std::vector<IVisibleObject>();
}

Segment::Ptr WorldState::getCurrentSegment() const{
    return _currentSegment;
}

WorldState::ConstPtr WorldState::setObjects(std::vector<IVisibleObject::Ptr> obstacles) const
{
	std::cout << "WorldState: size of obstacle vector: "  << obstacles.size() << std::endl;
	return nullptr;
}


bool WorldState::isDirectAncestor(WorldState::ConstPtr newWorld) const
{
	auto info = newWorld->getVersionInfo();
	long currentId = _version->id;
	while (info && info->id != currentId) {
		info = info->parent;
	}
	if (info && info->id == currentId) {
		return true;
	}
	return false;
}

void WorldState::print() const{

}

int WorldState::getId() const{
	return _version->id;
}

WorldState::ConstPtr WorldState::mergeWorldState(std::deque<WorldState::ConstPtr> _oldStates, WorldState::ConstPtr& newState) const {

	// we check what stuff changed from the common Ancestor to this state and 
	WorldState::ConstPtr commonAncestor = findCommonAncestor(_oldStates, newState);
	
	// check what changed from ancestor to current TODO
	 
	
	
	return WorldState::ConstPtr(this);
}

WorldState::ConstPtr WorldState::findCommonAncestor(std::deque<WorldState::ConstPtr> states, WorldState::ConstPtr newState) const
{
	std::vector<int> newParentIds;
	VersionInfo::ConstPtr version = newState->getVersionInfo();
	
	do
	{
		version = version->parent;
		newParentIds.push_back(version->id);
		
	}while(version->parent);

	std::vector<int> currentParentIds;
	version = getVersionInfo();
	do
	{
	      version = version->parent;
	      currentParentIds.push_back(version->id);
		
	}while(version->parent);

	std::vector<int> intersection;
	std::sort(newParentIds.begin(),newParentIds.end());
	std::sort(currentParentIds.begin(), currentParentIds.end());
	std::set_intersection(newParentIds.begin(), newParentIds.end(), currentParentIds.begin(), currentParentIds.end(), back_inserter(intersection));	

	if(intersection.size() > 0)
	{
		int firstCommonAncestorId = intersection[intersection.size()-1];	
		for(WorldState::ConstPtr state: states)
		{
			if(state->getVersionInfo()->id == firstCommonAncestorId)
				return state;
		}
	}
	return WorldState::ConstPtr();
}

// helper stuff 
void WorldState::setupIds(const WorldState* parent) {
	_generator = VersionIdGenerator::getInstance();

	if (parent != NULL) {
		_version = _generator->getId(parent->_version);
	} else {
		_version = _generator->getId();
	}
}


WorldState::ConstPtr WorldState::WorldStateBuilder::persist(){
	return WorldState::ConstPtr(new WorldState(const_cast<WorldState*>(_parent),
				_roadSigns,
				_currentSegment,
				_detector));
}

void WorldState::WorldStateBuilder::addDetectedRoadSign(const SignResult toAdd){
	_roadSigns = _roadSigns.add(toAdd);
}

void WorldState::WorldStateBuilder::addLineDetector(LineDetector::Ptr result) {
	_detector = result;
}

void WorldState::WorldStateBuilder::setCurrentSegment(Segment::Ptr segment)
{
    _currentSegment = segment;
}