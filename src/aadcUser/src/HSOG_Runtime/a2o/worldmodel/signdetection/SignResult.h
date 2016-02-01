#pragma once

#include <boost/smart_ptr.hpp>
#include <vector>
#include "worldmodel/impl/RoadSign.h"

namespace A2O {

class SignResult {


public:
  	typedef boost::shared_ptr<SignResult> Ptr;
  	typedef boost::shared_ptr<const SignResult> ConstPtr;

	void addSign(const RoadSign& sign)
	{
		_detectedSigns.push_back(sign);
	}

	std::vector<RoadSign> getSigns()
	{
		return _detectedSigns;
	}
private:
	std::vector<RoadSign> _detectedSigns;
};
}
