#pragma once

#include "IEventLogger.h"

namespace A2O {
	class DummyEventLogger: public virtual IEventLogger {

		public:

			virtual void logDecodeEvent(uint32_t timestamp, std::string name, double value) const {};
			virtual void logPose(const std::string& name, const Pose2D& pose) const {};
			virtual void logTimeTaken(const std::string& name, long duration) const {};

			
			virtual void stop() const {};
			virtual void start() const {};

			virtual ~DummyEventLogger(){};

	};
}
