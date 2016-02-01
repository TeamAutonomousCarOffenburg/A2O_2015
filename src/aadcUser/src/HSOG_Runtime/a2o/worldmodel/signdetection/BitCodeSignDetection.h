#pragma once

#include <aruco/highlyreliablemarkers.h>
#include <aruco/markerdetector.h>

#include <mutex>

#include "../ISignDetection.h"

#include "utils/concurrency/Worker.h"

namespace A2O {

/**
 * Detects Bitcode based signs from a camerasensor. 
 *
 */
class BitCodeSignDetection : public virtual ISignDetection, public Worker {

private:
	aruco::MarkerDetector _detector;
	aruco::Dictionary _Dictionary;

	const static std::string _markersYML;
	
	aruco::CameraParameters _cameraParameters;

	const double _signSizeM = 0.12;
	cv::Mat _frame;

	cv::Mat _cameraMatrix;
  	void init();
  	void start();
  	void stop();
  	void notifyPossibleNewFrame();
	void setResult(const SignResult& result); 

protected:
  void run();

public:
  virtual ~BitCodeSignDetection(){};

  void detectSigns();

  BitCodeSignDetection(){
	  init();
  }
 
  SignResult::Ptr getSignResult();
  void setFrame(cv::Mat& frame);
  cv::Mat getFrame();

  bool _debug = false;

  SignResult::Ptr _result;

  std::mutex _resultLock;
  std::mutex _updateFrame;
};

}
