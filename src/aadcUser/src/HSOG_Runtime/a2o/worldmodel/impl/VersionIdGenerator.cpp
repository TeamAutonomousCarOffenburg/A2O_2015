#include "VersionIdGenerator.h"

VersionInfo::ConstPtr VersionIdGenerator::getId(VersionInfo::ConstPtr parent) {
	long newId = _counter++;
	return boost::make_shared<const VersionInfo>(parent, newId);
}

VersionInfo::ConstPtr VersionIdGenerator::getId() {
	long newId = _counter++;
	return boost::make_shared<const VersionInfo>(newId);
}

VersionIdGenerator* VersionIdGenerator::_instance = NULL;
