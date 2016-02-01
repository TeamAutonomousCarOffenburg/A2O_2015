#pragma once

#include "../ISignDetection.h"

#include <boost/smart_ptr.hpp>
#include <iostream>

namespace A2O {

/**
 * does nothing 
 *
 */
class DummySignDetection : public virtual ISignDetection {

private:
  	void init(){};
  	void start(){};
  	void stop(){};
  	void notifyPossibleNewFrame(){};

public:
  virtual ~DummySignDetection(){};

  void detectSigns(){};
  SignResult::Ptr getSignResult(){
	return SignResult::Ptr();
  }

  DummySignDetection(){
	std::cout << "dummy " << std::endl;
  }
 
  void setFrame(cv::Mat& frame){};


};
}
