#pragma once

#include <opencv2/core/core.hpp>

#include "LineDetector.h"
#include "ScanLine.h"
#include "../ILaneDetection.h"

#include "utils/concurrency/Worker.h"

class LaneDetection: public virtual A2O::ILaneDetection, public A2O::Worker{

	public:
		LaneDetection()
		 {
		  _height = 480;
		  _width = 640;
		}
		
		void detectLanes();
		void detectInitialAlignment();
		
		// worker
		void init();
		void start();
		void stop();
		void notifyPossibleNewFrame();
		void setFrame(cv::Mat& frame, bool initialAlign);
		
		cv::Mat detectEdges(cv::Mat& frame);
		A2O::LineDetector::Ptr getLaneResult();

	private:
	  	void run();
		cv::Mat _frame;
		std::mutex _updateFrame;
		std::mutex _resultLock;
		cv::Mat getFrame();
		void setLaneResult(const A2O::LineDetector& result);
		A2O::LineDetector::Ptr _result;	
		
		int _width;
		int _height;
		bool _initialAlignment;
		
		int _CANNY_MIN_TRESHOLD = 150;
		int _CANNY_MAX_TRESHOLD = 250;
	
};

