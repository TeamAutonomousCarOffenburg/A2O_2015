#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/atomic.hpp>

#include <chrono>
#include <utility>

class VersionInfo {
	public:
		typedef boost::shared_ptr<const VersionInfo> ConstPtr;

		VersionInfo::ConstPtr parent;
		long id;
		std::chrono::system_clock::time_point time;

		VersionInfo(VersionInfo::ConstPtr p, long id) {
			this->parent = p;
			this->id = id;
			time = std::chrono::system_clock::now();

		}
		VersionInfo(long id) { this->id = id; }
	
};

class VersionIdGenerator {
	private:
		boost::atomic_long _counter;
		static VersionIdGenerator* _instance;
		VersionIdGenerator() { _counter = 0; }
		VersionIdGenerator(const VersionInfo&){};

	public:
		VersionInfo::ConstPtr getId();
		VersionInfo::ConstPtr getId(VersionInfo::ConstPtr parent);

		static VersionIdGenerator* getInstance() {
			if (!_instance) {
				_instance = new VersionIdGenerator();
			}
			return _instance;
		}
};

