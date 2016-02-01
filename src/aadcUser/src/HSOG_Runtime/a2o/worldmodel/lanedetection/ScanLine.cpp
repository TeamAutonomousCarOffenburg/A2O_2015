/*
 * ScanLine.cpp
 *
 *  Created on: 14.03.2015
 *      Author: kdorer
 */

#include "ScanLine.h"
#include <iostream>

using namespace A2O;
using namespace std;
using namespace cv;

ScanLine::ScanLine(int x1, int y1, int x2, int y2) : _line(x1, y1, x2, y2) {
	_line.getLinePoints(_points);
	currentPoint = 0;
}

ScanLine::~ScanLine() {
}

LineDetection ScanLine::findLine(Mat& edges, Rect& area, bool startAnew,
		float minDistance, float maxDistance, Eigen::Vector3d distPoint,
		Angle slopeLimit, bool isMaxSlopeLimit, int direction)
{
	LineDetection result;
	int angle = 0;
	bool slopeBad = false;
	float distancex = 0;
	do {
		if (startAnew) {
			result = findFirstLane(edges, area, direction);
			startAnew = false;
		} else {
			result = findNextLane(edges, area, direction);
		}

		if (result.valid) {
			distancex = result.startK(0) - distPoint(0);
			if (distancex > maxDistance) {
				// we will not find a line in range
				result.valid = false;
			}

			angle = (int) result.line.getSlope().deg();
			if (isMaxSlopeLimit) {
				slopeBad = abs(angle) > slopeLimit.deg();
			} else {
				slopeBad = abs(angle) < slopeLimit.deg();
			}
		}

	} while (result.valid && (slopeBad || distancex < minDistance));

	return result;
}

LineDetection ScanLine::findFirstLane(Mat& mat, Rect& area, int direction, bool bothWays) {
	currentPoint = 0;
	return findNextLane(mat, area, direction, bothWays);
}

LineDetection ScanLine::findNextLane(Mat& mat, Rect& area, int direction, bool bothWays) {
	LineDetection resultLane;
	Line slopeLine;

	// skip already searched points
	int i = 0;
	auto it = _points.begin();
	for (; it != _points.end() && i < currentPoint; ++it) {
		i++;
	}

	// search for lines
	for (; it != _points.end() && !resultLane.valid; ++it) {
		resultLane = findStraightLine(mat, area, *it, direction, slopeLine);
		currentPoint++;
	}
	if (resultLane.valid) {
		resultLane.scanCutP = resultLane.line.start;
		if (bothWays) {
			// search from scan point to the other direction
			slopeLine = Line(resultLane.line.end, resultLane.line.start);
			LineDetection rightLane2 = findStraightLine(mat, area, resultLane.line.start, ((direction + 4) % 8), slopeLine);
			if (rightLane2.valid) {
				// append the two lines
				resultLane.line.start = rightLane2.line.end;
				resultLane.startK = rightLane2.endK;
			}
		}
	}

	return resultLane;
}

LineDetection ScanLine::findStraightLine(Mat& mat, Rect& area, Point scanStart, int direction, Line slopeLine) {

	LineDetection result;
	int minLineLength = 7;
	Point cp(scanStart.x, scanStart.y);

	if (!area.contains(cp)) {
		// outside range
		return result;
	}

	int lastmx = cp.x;
	int lastmy = cp.y;
	bool hadSlope = slopeLine.isValid();
	bool pixAt =  mat.at<uchar>(cp.y, cp.x) >= 255;

	int count = 0;
	bool badslope = false;
	int slopeDeviation = 0;
	while (area.contains(cp) && pixAt) {
		Point nextPoint = cp;
		pixAt = gotoNextPoint(mat, nextPoint, direction);
		if (area.contains(nextPoint)) {
			cp = nextPoint;
			count++;
			if (count % minLineLength == 0) {
				if (count < minLineLength + 1 && !hadSlope) {
					// the first n pixels detected
					slopeLine = Line(scanStart, cp);
					lastmx = cp.x;
					lastmy = cp.y;
				} else {
					if (count < minLineLength * 2 + 1 && !hadSlope) {
						// we take the first two pieces
						slopeLine = Line(scanStart, cp);
					}
					// check if still straight
					int newSlopeDeviation = slopeLine.checkSameDirection(lastmx, lastmy, cp.x, cp.y, Angle::deg(10));
					if (abs(newSlopeDeviation + slopeDeviation) > 1) {
						// twice in a row in same direction
						badslope = true;
						break;
					} else if (newSlopeDeviation == 0) {
						// we have no violation
						lastmx = cp.x;
						lastmy = cp.y;
					}
					slopeDeviation = newSlopeDeviation;
				}
			}
		} else {
			// we have left the area
			pixAt = false;
		}
	}

	if (slopeDeviation != 0) {
		// we had a deviation before ending
		cp.x = lastmx;
		cp.y = lastmy;
	}

	if (count > minLineLength) {
		result.line.start.x = scanStart.x;
		result.line.start.y = scanStart.y;
		result.line.end.x = cp.x;
		result.line.end.y = cp.y;
		result.endedBadSlope = badslope;
		result.valid = true;
		result.startK = pixelToCamera(scanStart);
		result.endK = pixelToCamera(cp);
	}

	return result;
}

CurveDetection ScanLine::findRightCurve(Mat& mat, Point scanStart) {

	vector<Point> points;
	// we start to search left top and continue right top
	getCurvePoints(mat, scanStart, 1, points);
	Point nextStart = scanStart;
	if (points.size() > 0) {
		nextStart = points.at(points.size() - 1);
	}
	getCurvePoints(mat, nextStart, 7, points);

	if (points.size() < 15) {
		// invalid curve
		CurveDetection result;
		return result;
	}

	if (points.size() < 30) {
		// could be a left curve so try to find more pixels
		getCurvePoints(mat, points.at(points.size() - 1), 1, points);
	}

	return setCurveResult(points);
}

CurveDetection ScanLine::findLeftCurve(Mat& mat, Point scanStart) {

	CurveDetection result;
	vector<Point> points;
	getCurvePoints(mat, scanStart, 7, points);
	Point nextStart = scanStart;
	if (points.size() > 0) {
		nextStart = points.at(points.size() - 1);
	}
	getCurvePoints(mat, nextStart, 1, points);

	if (points.size() < 8) {
		// invalid curve
		return result;
	}

	return setCurveResult(points);
}

CurveDetection ScanLine::setCurveResult(vector<Point> points) {
	CurveDetection result;
	result.valid = true;
	result.start = points.at(0);
	result.startK = pixelToCamera(result.start);
	result.middle = points.at(points.size() / 2);
	result.middleK = pixelToCamera(result.middle);
	result.end = points.at(points.size() - 1);
	result.endK = pixelToCamera(result.end);
	// result.calculateAlternativeMiddle();
	result.calculateMiddle();
	// result.calculateCurveRadius();
	if (result.radius < 0.2 || result.radius > 4) {
		// looks more like a straight line
		result.valid = false;
	}
	return result;
}

void ScanLine::getCurvePoints(Mat& mat, Point scanStart, int direction, vector<Point>& points) {

	Point cp(scanStart.x, scanStart.y);
	if (cp.x < 1 || cp.x > 638 || cp.y < 1 || cp.y > 238) {
		// outside range
		return;
	}

	bool pixAt =  mat.at<uchar>(cp.y, cp.x) >= 255;

	int count = 0;
	while ((cp.x > 0) && (cp.x < _cfg.width - 1) && (cp.y > 0) && (cp.y < _cfg.height / 2 - 1) && pixAt) {
		pixAt = gotoNextPoint(mat, cp, direction);
		count++;
		if (pixAt) {
			points.push_back(Point(cp));
		}
	}
}

bool ScanLine::gotoNextPoint(Mat& mat, Point& currentPoint, int direction)
{
	int x = currentPoint.x;
	int y = currentPoint.y;
	bool nextPixel = mat.at<uchar>(y + dirDelta[direction][1], x + dirDelta[direction][0]) >= 255;
	if (nextPixel) {
		x += dirDelta[direction][0];
		y += dirDelta[direction][1];
	} else {
		nextPixel = mat.at<uchar>(y + dirDelta[direction][3], x + dirDelta[direction][2]) >= 255;
		if (nextPixel) {
			x += dirDelta[direction][2];
			y += dirDelta[direction][3];
		} else {
			nextPixel = mat.at<uchar>(y + dirDelta[direction][5], x + dirDelta[direction][4]) >= 255;
			if (nextPixel) {
				x += dirDelta[direction][4];
				y += dirDelta[direction][5];
			}
		}
	}
	if (nextPixel) {
		currentPoint.x = x;
		currentPoint.y = y;
	}
	return nextPixel;
}

Eigen::Vector3d ScanLine::pixelToCamera(Point pixel)
{
	int y = -pixel.y;
	if (y == 0) {
		y = 1;
	}

	float xf = (-_cfg.heightCamera * _cfg.focalVertical) / y;
	float yf = (pixel.x - _cfg.xCenter) * xf / _cfg.focalHorizontal;

	Eigen::Vector3d result(xf / 1000.0, -(yf / 1000.0), 0);
	return result;
}