#include "openCvLane.h"
#include "RoadStatus.h"
#include <opencv2/imgproc.hpp>

using namespace A2O;

void LaneDetection::run() {
	while (!m_stop) {
		detectLanes();
	}
}

void LaneDetection::setFrame(cv::Mat& frame, bool initialAlign) {
	_updateFrame.lock();
	_frame = frame;
	_initialAlignment = initialAlign;
	_updateFrame.unlock();
}

cv::Mat LaneDetection::getFrame() {
	_updateFrame.lock();
	cv::Mat ret(_frame);
	_updateFrame.unlock();
	return ret;
}

void LaneDetection::start() {
	Runnable::start("LaneDetection");
}

void LaneDetection::stop() {
	Runnable::stop();
}

void LaneDetection::notifyPossibleNewFrame() {
	Worker::notify();
}

// for use with model
void LaneDetection::detectLanes() {
	static void* oldFrameDataLocation = nullptr;

	if (!wait2())
		return;

	cv::Mat frame = getFrame();
	if (frame.empty() || frame.data == oldFrameDataLocation)
		return;

	oldFrameDataLocation = frame.data;

	// image conversions and edge detection
	cv::Mat edges = detectEdges(frame);

	LineDetector detector;
	if (_initialAlignment) {
		detector.initialDetection(edges);
	} else {
		detector.detectLanes(edges);
	}

	setLaneResult(detector);
}

void LaneDetection::setLaneResult(const LineDetector& result) {
	_resultLock.lock();
	_result = boost::make_shared<LineDetector>(result);
	_resultLock.unlock();
}

LineDetector::Ptr LaneDetection::getLaneResult() {
	_resultLock.lock();
	LineDetector::Ptr res = _result;
	_result = LineDetector::Ptr();
	_resultLock.unlock();
	return res;
}

cv::Mat LaneDetection::detectEdges(cv::Mat& frame) {
	cv::Size frame_size = cv::Size(_width, _height / 2);
	cv::Mat edges(frame_size, CV_8UC1);
	cv::Mat grey(frame_size, CV_8UC1);
	// cut off upper half
	cv::Rect rect(0, frame_size.height, frame_size.width, frame_size.height);
	cv::Mat temp_frame = frame(rect);
	cv::cvtColor(temp_frame, grey, cv::COLOR_BGR2GRAY); // convert to grayscale
	
	// Perform a Gaussian blur ( Convolving with 5 X 5 Gaussian) & detect edges
	cv::GaussianBlur(grey, grey, cv::Size(5, 5), 0);
	
	cv::Canny(grey, edges, _CANNY_MIN_TRESHOLD, _CANNY_MAX_TRESHOLD);
	
	return edges;
}