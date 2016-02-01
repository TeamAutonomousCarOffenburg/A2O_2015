/*
 * ScanLine.h
 *
 *  Created on: 14.03.2015
 *      Author: kdorer
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include "Detection.h"
#include "Line.h"
#include "utils/value/FuzzyCompare.h"

namespace A2O {

class LineDetection : public Detection
{
public:
	Line line;
	cv::Point scanCutP;
	bool endedBadSlope = false;

	float getSameStartDistancePlausibility(LineDetection& other, float goodDeviation, float badDeviation) {
		if (!valid || !other.valid) {
			return 0.0;
		}
		return getSameDistancePlausibility(startK, other.startK, goodDeviation, badDeviation);
	}

	float getSameEndDistancePlausibility(LineDetection& other, float goodDeviation, float badDeviation) {
		if (!valid || !other.valid) {
			return 0.0;
		}
		return getSameDistancePlausibility(endK, other.endK, goodDeviation, badDeviation);
	}

	void invert()
	{
		cv::Point temp = line.start;
		line.start = line.end;
		line.end = temp;

		Eigen::Vector3d temp2 = startK;
		startK = endK;
		endK = temp2;
	}

private:
};
} /* namespace A2O */

