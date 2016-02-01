/*
 * ScanLine.h
 *
 *  Created on: 14.03.2015
 *      Author: kdorer
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "LaneDetectionConfiguration.h"
#include "Line.h"
#include "LineDetection.h"
#include "CurveDetection.h"


namespace A2O {

class ScanLine {
public:
	ScanLine(Line& line);
	ScanLine(int x1, int y1, int x2, int y2);
	virtual ~ScanLine();

	/**
	 * Tries to find a line that is on this scanline and has the spec fullfiled
	 * edges: the image in which to search
	 * area: a rectangle (in pixel coordinates lower half image) inside which to search
	 * startAnew: true if findFirstLane should be called the first time
	 * minDistance: the minimum x distance (camera coordinates)
	 * 	the start coordinate of the line should have to the passed distPoint
	 * maxDistance: the maximum x distance (camera coordinates)
	 * 	the start coordinate of the line should have to the passed distPoint
	 * slopeLimit: the maximal angle the line may have
	 * isMaxSlopeLimit: if true this is the maximum slope, false the minimum slope
	 * direction: the direction into which to search (0-7)
	 */
	LineDetection findLine(cv::Mat& edges, cv::Rect& area, bool startAnew,
			float minDistance, float maxDistance, Eigen::Vector3d distPoint,
			Angle slopeLimit, bool isMaxSlopeLimit, int direction);

	/**
	 * mat: the image in which to search
	 * area: a rectangle (in pixel coordinates lower half image) inside which to search
	 * direction: sector in which we follow the line // TODO convert int to enum
	 */
	LineDetection findFirstLane(cv::Mat& mat, cv::Rect& area, int direction, bool bothWays = true);
	LineDetection findNextLane(cv::Mat& mat, cv::Rect& area, int direction, bool bothWays = true);

	/**
	 * mat: the image in which to search
	 * area: a rectangle (in pixel coordinates lower half image) inside which to search
	 * scanStart: the point from which to start searching
	 * direction: sector in which we follow the line // TODO convert int to enum
	 * slopeLine: a line defining the desired slope (if valid)
	 */
	LineDetection findStraightLine(cv::Mat& mat, cv::Rect& area, cv::Point scanStart,
			int direction, Line slopeLine = Line());

	CurveDetection findLeftCurve(cv::Mat& mat, cv::Point scanStart);
	CurveDetection findRightCurve(cv::Mat& mat, cv::Point scanStart);

	/**
	 * Converts the passed points pixel coordinates to 3D camera coordinate system
	 * TODO appart from cfg this is a static method: move somewhere else
	 */
	Eigen::Vector3d pixelToCamera(cv::Point pixel);

private:
	LaneDetectionConfiguration _cfg;
	std::vector<cv::Point> _points;
	Line _line;
	int currentPoint;
	int dirDelta[8][6] =  { //
			{0, -1, -1, -1, 1, -1}, // up
			{-1, -1, -1, 0, 0, -1}, // left up
			{-1, 0, -1, 1, -1, -1}, // left
			{-1, 1, 0, 1, -1, 0}, // left down
			{0, 1, 1, 1, -1, 1}, // down
			{1, 1, 1, 0, 0, 1}, // right down
			{1, 0, 1, -1, 1, 1}, // right
			{1, -1, 0, -1, 1, 0}  // right up
	};

	bool gotoNextPoint(cv::Mat& mat, cv::Point& currentPoint, int direction);
	void getCurvePoints(cv::Mat& mat, cv::Point scanStart, int direction, std::vector<cv::Point>& points);
	CurveDetection setCurveResult(std::vector<cv::Point> points);
};

} /* namespace A2O */

