/*
 * ScanLine.cpp
 *
 *  Created on: 14.03.2015
 *      Author: kdorer
 */

#include "LineDetector.h"
#include <iostream>

using namespace cv;
using namespace Eigen;

namespace A2O {

LineDetector::LineDetector() {
	nextSegment = NONE;
	nextSegmentDistance = 0;
	fullImage = Rect(1, 1, 638, 238);
}

LineDetector::~LineDetector() {
}

void LineDetector::initialDetection(Mat& edges)
{
	// try to find opposite lane
	ScanLine oppositeScan(320, 240, 320, 10);
	Vector3d distPoint(0,0,0);
	stopLineMiddle = oppositeScan.findLine(edges, fullImage, true, -1000, 2, distPoint, Angle::deg(10), true, 6);
	if (stopLineMiddle.valid) {
		upperRightStart = oppositeScan.findLine(edges, fullImage, false, 0.2, 2, stopLineMiddle.startK, Angle::deg(10), true, 6);
	}

	// try to find right lane
	ScanLine rightScan(100, 180, 500, 180);
	rightLine = rightScan.findLine(edges, fullImage, true, -1000, 2, distPoint, Angle::deg(90), true, 7);

	Vector3d start = Vector3d(0,0,0);
	Vector3d end = Vector3d(0,0,0);
	// TODO add more tests
	if (rightLine.valid) {
		initialAlignment.valid = true;
		initialAlignment.parallelParking = true;
		start = rightLine.startK;
		end = rightLine.endK;
	} else {
		initialAlignment.valid = true;
		initialAlignment.parallelParking = false;
		if (upperRightStart.valid) {
			start = upperRightStart.startK;
			end = upperRightStart.endK;

			if (stopLineMiddle.valid) {
				float l1 = stopLineMiddle.getLength();
				float l2 = upperRightStart.getLength();
				if (l1 > l2) {
					start = stopLineMiddle.startK;
					end = stopLineMiddle.endK;
				}
			}
		} else if (stopLineMiddle.valid) {
			start = stopLineMiddle.startK;
			end = stopLineMiddle.endK;
		} else {
			// we do not see any lines :-(
			initialAlignment.valid = false;
		}
	}
	initialAlignment.startposition = Vector2d(start(0), start(1));
	initialAlignment.endposition = Vector2d(end(0), end(1));
}


void LineDetector::detectLanes(Mat& edges) {
	init();

	// right driving lane
	detectRightSide(edges);

	// find left lane
	detectLeftSide(edges);

	// identify upcoming segment
	detectNextSegment();

	// calculate the pose of the detected segment
	if (nextSegment == XCROSSROAD || nextSegment == TLEFT || nextSegment == TRIGHT || nextSegment == TLEFTRIGHT) {
		setNextSegmentPoseCrossing();
	} else if (nextSegment == STRAIGHT) {
		setNextSegmentPoseStraight();
	}
}

void LineDetector::init() {
	rightLine.valid = false;
	stopLine.valid = false;
	stopLineMiddle.valid = false;
	startRight.valid = false;
	endRight.valid = false;
	stopRight.valid = false;
	stopRightMiddle.valid = false;
	startLeft.valid = false;
	leftLine.valid = false;
	endLeft.valid = false;
	upperRight.valid = false;
	upperRightStart.valid = false;
	upperLeft.valid = false;
	upperLeftStart.valid = false;
	leftCurve.valid = false;
	rightCurve.valid = false;
}

void LineDetector::detectRightSide(Mat& edges) {
	// right driving lane
	int y = _cfg.yOffset - _cfg.rightLaneStartPos;
	ScanLine rightScan(340, y, _cfg.width, y);
	rightLine = rightScan.findFirstLane(edges, fullImage, 1);
	if (!rightLine.valid) {
		// alternative scan line for right lane
		ScanLine rightScan2(350, 75, 450, 75);
		rightLine = rightScan2.findFirstLane(edges, fullImage, 1);
	}
	if (rightLine.valid) {
		// find stop line
		ScanLine stopScan(rightLine.line.start.x - 20, rightLine.line.start.y,
				rightLine.line.end.x - 30, rightLine.line.end.y - 10);
		stopLine = stopScan.findLine(edges, fullImage, true, -1000, 1000, rightLine.startK, Angle::deg(172), false, 2);
		if (stopLine.valid) {
			ScanLine midlineScan(stopLine.line.end.x, stopLine.line.end.y + 1,
					stopLine.line.end.x - 7, stopLine.line.end.y + 1);
			stopLineMiddle = midlineScan.findFirstLane(edges, fullImage, 3);
		} else if (rightLine.endedBadSlope) {
			// find right curve
			rightCurve = rightScan.findRightCurve(edges, rightLine.line.end);
		}

		// find end right
		ScanLine rightScanCrossing(rightLine.line.end.x + 2, rightLine.line.end.y,
				rightLine.line.end.x + 10, 20);
		Rect endRightRect(rightLine.line.end.x - 2, 1, 638 - rightLine.line.end.x, 238);
		endRight = rightScanCrossing.findLine(edges, endRightRect, true, -1000, 0.25, rightLine.endK, Angle::deg(8), true, 6);

		// find right upper part of crossing
		detectRightUpperLines(edges, rightScanCrossing);

		// find start right
		startRight = stopScan.findStraightLine(edges, fullImage, rightLine.line.start, 6);
		// prevent to detect right line again
		if (startRight.valid) {
			if (abs(startRight.line.getSlope().deg()) > 15) {
				// invalid direction
				startRight.valid = false;
			}
		}
	}
}

void LineDetector::detectRightUpperLines(Mat& edges, ScanLine& rightScanCrossing)
{
	Vector3d rightCornerK = rightLine.endK;
	if (endRight.valid) {
		rightCornerK = endRight.startK;
	}
	if (endRight.valid || stopLine.valid
			|| (rightCornerK(0) < 3.0 && !rightLine.endedBadSlope)) {
		// if right line ends earlier than expected continue with middle line to the right
		stopRightMiddle = rightScanCrossing.findLine(edges, fullImage,
				!endRight.valid, 0.2, 1.3, rightCornerK, Angle::deg(10), true,
				6);
		if (!stopRightMiddle.valid) {
			// we did not find stop line middle, try a second scan line
			ScanLine rightScan2(rightLine.line.end.x, rightLine.line.end.y - 2,
					rightLine.line.end.x - 10, 5);
			stopRightMiddle = rightScan2.findLine(edges, fullImage, true,
					0.2, 1.3, rightCornerK, Angle::deg(10), true, 6);
		}

		if (stopRightMiddle.valid) {
			// could be there is no middle stop line
			float distanceX = stopRightMiddle.startK(0) - rightCornerK(0);
			if (distanceX < 0.7) {
				stopRight = rightScanCrossing.findStraightLine(edges, fullImage,
						stopRightMiddle.line.start, 1);
				if (stopRight.valid) {
					upperRightStart = rightScanCrossing.findStraightLine(edges,
							fullImage, stopRight.line.end, 2);
					if (upperRightStart.valid) {
						// we have to change the direction to be consistent
						upperRightStart.invert();
						upperRight = rightScanCrossing.findStraightLine(edges,
								fullImage, upperRightStart.line.start, 1);
					}
				} else {
					// there is no stop line, so try to find upper with old scanline
					upperRightStart = rightScanCrossing.findLine(edges,
							fullImage, false, 0.8, 1.3, rightCornerK,
							Angle::deg(10), true, 6);
					if (upperRightStart.valid) {
						upperRight = rightScanCrossing.findStraightLine(edges,
								fullImage, upperRightStart.line.start, 1);
					}
				}
			} else if (distanceX >= 0.7 && distanceX <= 1.3) {
				// we found the upper right lane
				upperRightStart = stopRightMiddle;
				stopRightMiddle.valid = false;
				if (upperRightStart.valid) {
					upperRight = rightScanCrossing.findStraightLine(edges,
							fullImage, upperRightStart.line.start, 1);
				}
			}
		}
	}

	if (stopLine.valid) {
		// look for line straight ahead
		int x = (int) ((stopLine.line.start.x + stopLine.line.end.x) * 0.5);
		ScanLine oppositeScan(x, stopLine.line.start.y - 5, x, 20);
		LineDetection oppositeLine = oppositeScan.findLine(edges, fullImage, true, 0.75, 1.6,
				stopLine.startK, Angle::deg(172), false, 2);
		if (oppositeLine.getHorizontalBlockingPlausibility() > 0.5) {
			// we probably have a TLEFTRIGHT
			upperRightStart = oppositeLine;
		}
	}
}



void LineDetector::detectLeftSide(Mat& edges) {
	// find left lane
	ScanLine leftScan(8, 180, 8, 50);
	leftLine = leftScan.findFirstLane(edges, fullImage, 7);
	if (leftLine.valid) {
		int angle = (int) leftLine.line.getSlope().deg();
		// did we find left start?
		if (abs(angle) < 6) {
			// take it as leftStart
			leftLine.valid = false;
			startLeft = leftScan.findFirstLane(edges, fullImage, 6);
			if (startLeft.valid) {
				startLeft.invert();
				leftLine = leftScan.findStraightLine(edges, fullImage, startLeft.line.start, 7);
			}
		}
	}
	if (leftLine.valid) {
		// do we have a curve?
		int angle = (int) leftLine.line.getSlope().deg();
		if (abs(angle) < 15 && leftLine.getLength() > 0.8) {
			// find left curve
			leftCurve = leftScan.findLeftCurve(edges, leftLine.line.start);
			if (!rightCurve.valid) {
				rightCurve = leftScan.findRightCurve(edges, rightLine.line.end);
			}

		} else {
			// find end left
			ScanLine leftScan(leftLine.line.end.x - 2, leftLine.line.end.y - 3,
					leftLine.line.end.x - 2, leftLine.line.end.y);
			endLeft = leftScan.findLine(edges, fullImage, true,
					-1000, 0.3, leftLine.endK, Angle::deg(172), false, 2);
			if (endLeft.valid) {
				// we have a left crossing. find upper part
				ScanLine leftUpperScan(endLeft.line.start.x, endLeft.line.start.y - 3,
						endLeft.line.start.x, 20);

				// find left stopline
				stopLeft = leftUpperScan.findLine(edges, fullImage, true,
						-1000, 0.75, endLeft.startK, Angle::deg(90), true, 7);

				stopLeftMiddle = leftUpperScan.findLine(edges, fullImage, true,
						0.25, 0.75, endLeft.startK, Angle::deg(170), false, 2);

				upperLeftStart = leftUpperScan.findLine(edges, fullImage, true,
						0.7, 1.3, endLeft.startK, Angle::deg(170), false, 2);
				if (!upperLeftStart.valid) {
					// could be that endLeft is really stopLeftMiddle
					// search again in closer area
					upperLeftStart = leftUpperScan.findLine(edges, fullImage, true,
							0.2, 0.71, endLeft.startK, Angle::deg(170), false, 2);
				}

				if (upperLeftStart.valid && upperLeftStart.line.start.x < 280) {
					// looks like non T crossing, so try to find upper left lane
					upperLeft = leftUpperScan.findStraightLine(edges, fullImage,
							upperLeftStart.line.start, 7);
				}
			} else {
				// find left curve
				leftCurve = leftScan.findLeftCurve(edges, leftLine.line.end);
			}
		}
	}
}

void LineDetector::setNextSegmentDistanceCrossing() {
	// we have a crossing
	if (stopLine.valid) {
		nextSegmentDistance = stopLine.endK(0);
	} else if (endRight.valid) {
		nextSegmentDistance = endRight.startK(0);
		if (endLeft.valid && endLeft.startK(0) < nextSegmentDistance) {
			nextSegmentDistance = endLeft.startK(0);
		}
	} else if (endLeft.valid) {
		nextSegmentDistance = endLeft.startK(0);
	} else if (upperRightStart.valid) {
		nextSegmentDistance = upperRightStart.startK(0) - 1.0;
	} else if (upperLeftStart.valid) {
		nextSegmentDistance = upperLeftStart.startK(0) - 1.0;
	}
}

void LineDetector::setNextSegmentPoseCrossing() {

	// we have a crossing
	if (nextSegment == TLEFT) {
		if (leftLine.valid) {
			// use end of left line
			Vector2d start = Vector2d(leftLine.startK(0), leftLine.startK(1));
			Vector2d end = Vector2d(leftLine.endK(0), leftLine.endK(1));
			Vector2d dir = (end - start);
			dir.normalize();
			Vector2d orthogonal = Angle::Deg_N90() * dir;
			orthogonal.normalize();
			Vector2d pos = end + orthogonal * 0.45;
			Angle angle = Angle::to(dir);
			nextSegmentPose = Pose2D(pos, angle);
		}
		return;
	}

	if (stopLine.valid) {
		Vector2d start = Vector2d(stopLine.startK(0), stopLine.startK(1));
		Vector2d end = Vector2d(stopLine.endK(0), stopLine.endK(1));
		Vector2d dir = (end - start);
		dir.normalize();
		Vector2d orthogonal = Angle::Deg_N90() * dir;
		orthogonal.normalize();
		Vector2d pos = start + dir * 0.45 + orthogonal * 0.15;
		Angle angle = Angle::to(orthogonal);
		nextSegmentPose = Pose2D(pos, angle);
		return;
	}

	if (rightLine.valid) {
		// use end of right line
		Vector2d start = Vector2d(rightLine.startK(0), rightLine.startK(1));
		Vector2d end = Vector2d(rightLine.endK(0), rightLine.endK(1));
		Vector2d dir = (end - start);
		dir.normalize();
		Vector2d orthogonal = Angle::Deg_90() * dir;
		orthogonal.normalize();
		Vector2d pos = end + orthogonal * 0.45;
		Angle angle = Angle::to(dir);
		nextSegmentPose = Pose2D(pos, angle);
	}
}

void LineDetector::setNextSegmentPoseStraight() {

	if (rightLine.valid) {
		Vector2d start = Vector2d(rightLine.startK(0), rightLine.startK(1));
		Vector2d end = Vector2d(rightLine.endK(0), rightLine.endK(1));
		Vector2d dir = (end - start);
		dir.normalize();
		Vector2d orthogonal = Angle::Deg_90() * dir;
		orthogonal.normalize();
		Vector2d pos = start + orthogonal * 0.45;
		Angle angle = Angle::to(dir);
		nextSegmentPose = Pose2D(pos, angle);
		return;
	}

	if (leftLine.valid) {
		// use start of left line
		Vector2d start = Vector2d(leftLine.startK(0), leftLine.startK(1));
		Vector2d end = Vector2d(leftLine.endK(0), leftLine.endK(1));
		Vector2d dir = (end - start);
		dir.normalize();
		Vector2d orthogonal = Angle::Deg_N90() * dir;
		orthogonal.normalize();
		Vector2d pos = start + orthogonal * 0.45;
		Angle angle = Angle::to(dir);
		nextSegmentPose = Pose2D(pos, angle);
	}
}

void LineDetector::detectNextSegment() {

	nextSegment = NONE;

	// do we have a curve?
	bool haveCurve = false;
	if (leftLine.valid) {
		int angle = (int) leftLine.line.getSlope().deg();
		if (abs(angle) < 15 && leftLine.getLength() > 0.8) {
			haveCurve = true;
		}
	}

	if (rightLine.valid) {
		int angle2 = (int) rightLine.line.getSlope().deg();
		if (abs(angle2) > 165 && rightLine.getLength() > 0.8) {
			haveCurve = true;
		}
	}

	// is there a crossing?
	float pStop = stopLine.getLengthPlausibility(0.4, 0.1, 0.2);
	float pStopMiddle = stopLineMiddle.getMinLengthPlausibility(0.05, 0.15);
	float pEndRight = endRight.getMinLengthPlausibility(0.1, 0.2);
	float pStopRightMiddle = stopRightMiddle.getMinLengthPlausibility(0.05, 0.15);
	float pStopRight = stopRight.getMinLengthPlausibility(0.1, 0.2);
	float pUpperRight = upperRightStart.getMinLengthPlausibility(0.1, 0.2);
	float pUpperRightStart = upperRightStart.getMinLengthPlausibility(0.05, 0.15);

	float pEndLeft = endLeft.getMinLengthPlausibility(0.05, 0.10);
	float pStopLeftMiddle = stopLeftMiddle.getMinLengthPlausibility(0.05, 0.10);
	float pStopLeft = stopLeft.getMinLengthPlausibility(0.1, 0.2);
	float pUpperLeft = upperLeft.getMinLengthPlausibility(0.05, 0.15);
	float pUpperLeftStart = upperLeftStart.getMinLengthPlausibility(0.05, 0.15);

	float pStraightClosed1 = upperRightStart.getHorizontalBlockingPlausibility();
	float pStraightClosed2 = upperLeftStart.getHorizontalBlockingPlausibility();
	float pLeftEndsWhereRight = 0;
	float pRightLaneEnds = 0;
	if (endRight.valid && endRight.endK(0) < 3.0 && !endRight.endedBadSlope) {
		pRightLaneEnds = 1;
	} else if (stopLine.valid){
		pLeftEndsWhereRight = leftLine.getSameEndDistancePlausibility(rightLine, 0.6, 0.8);
	}

	float pHaveRight = pStop + pStopMiddle + pEndRight + pUpperRight + pUpperRightStart
			+ pStopRight + pStopRightMiddle + pStraightClosed1 + pStraightClosed2 + pRightLaneEnds;
	float pHaveLeft = pLeftEndsWhereRight + pEndLeft + pUpperLeft + pUpperLeftStart
			+ pStopLeft + pStopLeftMiddle + pStraightClosed1 + pStraightClosed2;

	if (!haveCurve && (pHaveRight + pHaveLeft > 1.2)) {
		bool haveRight = pHaveRight > 0.7;
		bool haveLeft = pHaveLeft > 0.7;

		// we have a crossing
		setNextSegmentDistanceCrossing();

		if (leftLine.valid) {
			float leftEndDist = leftLine.endK(0);
			if (leftEndDist - nextSegmentDistance > 1.0) {
				// have a long left lane
				nextSegment = TRIGHT;
				return;
			}
		}

		if (rightLine.valid) {
			float rightEndDist = rightLine.endK(0);
			if (rightEndDist - nextSegmentDistance > 1.0) {
				// have a long right lane
				nextSegment = TLEFT;
				return;
			}
		}

		if (haveRight) {

			if (!haveLeft) {
				// only have right lane
				nextSegment = TRIGHT;
				return;
			}

			if ((!endRight.valid && !upperRightStart.valid) || (!endLeft.valid && !upperLeftStart.valid)) {
				// we do not have left crossing lines, but if upper right start is blocking it is TLEFTRIGHT
				if (pStraightClosed1 + pStraightClosed2 > 0.8) {
					// road blocked ahead
					nextSegment = TLEFTRIGHT;
					return;
				}
			}

			float pSameDistance1 = endRight.getSameStartDistancePlausibility(endLeft, 0.6, 0.8);
			float pSameDistance2 = upperRightStart.getSameStartDistancePlausibility(upperLeftStart, 0.6, 0.8);
			if (pSameDistance1 + pSameDistance2 + pLeftEndsWhereRight > 0.7) {
				// we have left and right lanes
				if (pStraightClosed1 + pStraightClosed2 > 0.8) {
					// road blocked ahead
					nextSegment = TLEFTRIGHT;
					return;
				}

				// right and left lane are at same distance
				nextSegment = XCROSSROAD;
				return;

			} else {
				// the right and left lane seem not to be same distance
				// take the closer of the two
				float distance2 = endLeft.startK(0);
				if (!endLeft.valid) {
					distance2 = upperLeftStart.startK(0) - 1.0;
				}
				if (nextSegmentDistance > distance2) {
					nextSegment = TLEFT;
					nextSegmentDistance = distance2;
				} else {
					nextSegment = TRIGHT;
				}
				return;
			}
		} else {
			// only have left
			nextSegment = TLEFT;
		}

	} else {
		// no crossing
		CurveDetection curveToUse;
		if (rightCurve.valid && leftCurve.valid) {
			// both curves are valid, look at the longer one (in pixel coordinates)
			float len1 = rightCurve.getCurvePixelLength();
			float len2 = leftCurve.getCurvePixelLength();

			if (len2 > len1) {
				curveToUse = leftCurve;
			} else {
				curveToUse = rightCurve;
			}
		} else if (rightCurve.valid) {
			curveToUse = rightCurve;
		} else if (leftCurve.valid) {
			curveToUse = leftCurve;
		} else {
			// no valid curve, so straight
			nextSegment = STRAIGHT;
			return;
		}

		float radius = curveToUse.radius;
		bool small = (radius < 2.0);
		if (curveToUse.isLeft()) {
			if (small) {
				nextSegment = SCLEFT;
			} else {
				nextSegment = BCLEFT;
			}
		} else {
			if (small) {
				nextSegment = SCRIGHT;
			} else {
				nextSegment = BCRIGHT;
			}
		}
	}
}

} /* namespace A2O */
