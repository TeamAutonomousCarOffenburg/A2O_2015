#pragma once

#include "IActuator.h"
#include "maneuver/IManeuverConstants.h"

#include <boost/smart_ptr.hpp>

namespace A2O {

	class IManeuverStatusActuator : public virtual IActuator {
		public:
			typedef boost::shared_ptr<IManeuverStatusActuator> Ptr;
			typedef boost::shared_ptr<const IManeuverStatusActuator> ConstPtr;

			virtual ~IManeuverStatusActuator(){};
			virtual void setStatus(const ManeuverStatus& status) = 0;
			virtual const ManeuverStatus& getStatus() const = 0;
			virtual void setCurrentManeuver(const int& maneuverId) = 0;
			virtual const int& getCurrentManeuver() const = 0;

	};

}
