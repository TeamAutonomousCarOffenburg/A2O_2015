/*
 * ScanLine.h
 *
 *  Created on: 14.03.2015
 *      Author: kdorer
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "LaneDetectionConfiguration.h"
#include "ScanLine.h"
#include "RoadStatus.h"
#include "InitialAlignment.h"
#include "utils/geometry/Pose2D.h"


namespace A2O {


class LineDetector {
public:
	typedef boost::shared_ptr<LineDetector> Ptr;
	typedef boost::shared_ptr<const LineDetector> ConstPtr;
		
	LineDetector();
	virtual ~LineDetector();

	void initialDetection(cv::Mat& edges);
	void detectLanes(cv::Mat& edges);

	LineDetection rightLine;
	LineDetection stopLine;
	LineDetection stopLineMiddle;
	LineDetection startRight;
	LineDetection endRight;
	LineDetection stopRight;
	LineDetection stopRightMiddle;
	LineDetection startLeft;
	LineDetection leftLine;
	LineDetection endLeft;
	LineDetection stopLeft;
	LineDetection stopLeftMiddle;
	LineDetection upperRight;
	LineDetection upperRightStart;
	LineDetection upperLeft;
	LineDetection upperLeftStart;
	CurveDetection leftCurve;
	CurveDetection rightCurve;
	InitialAlignment initialAlignment;

	Pose2D nextSegmentPose;
	RoadStatus nextSegment;
	float nextSegmentDistance;

private:
	LaneDetectionConfiguration _cfg;
	cv::Rect fullImage;

	void detectRightSide(cv::Mat& edges);
	void detectRightUpperLines(cv::Mat& edges, ScanLine& rightScanCrossing);
	void detectLeftSide(cv::Mat& edges);
	void detectNextSegment();
	void init();
	void setNextSegmentDistanceCrossing();
	void setNextSegmentPoseCrossing();
	void setNextSegmentPoseStraight();
};

} /* namespace A2O */

