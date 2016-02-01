/*
 * ScanLine.h
 *
 *  Created on: 14.03.2015
 *      Author: kdorer
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include "Line.h"
#include "utils/value/FuzzyCompare.h"

namespace A2O {

class Detection
{
public:
	float getLengthPlausibility(float expectedLength, float goodDeviation, float badDeviation) {
		if (!valid) {
			return 0.0;
		}
		// evaluate fit to expected length
		float length = getLength();
		float delta = fabs(length - expectedLength);
		return FuzzyCompare::getLinearFuzzyValue(goodDeviation, badDeviation, false, delta);
	}

	float getMinLengthPlausibility(float absoluteMinLength, float goodMinLength) {
		if (!valid) {
			return 0.0;
		}
		float length = getLength();
		return FuzzyCompare::getLinearFuzzyValue(absoluteMinLength, goodMinLength, true, length);
	}

	float getSameDistancePlausibility(Eigen::Vector3d first, Eigen::Vector3d second,
			float goodDeviation, float badDeviation) {
		float dist1 = first.norm();
		float dist2 = second.norm();
		float delta = fabs(dist2 - dist1);
		return FuzzyCompare::getLinearFuzzyValue(goodDeviation, badDeviation, false, delta);
	}

	float getHorizontalBlockingPlausibility() {
		if (!valid) {
			return 0.0;
		}
		float left = 0.7;
		float right = - 0.2;
		if (startK(1) > left && endK(1) < right) {
			return 1.0;
		}
		if (startK(1) < right && endK(1) > left) {
			return 1.0;
		}
		return 0.0;
	}

	float getStartDistance() {
		return startK.norm();
	}

	float getEndDistance() {
		return endK.norm();
	}

	float getDistanceToStart(Eigen::Vector3d other) {
		return (startK-other).norm();
	}

	float getDistanceToEnd(Eigen::Vector3d other) {
		return (endK-other).norm();
	}

	float getLength() {
		Eigen::Vector3d len = endK - startK;
		return len.norm();
	}

	bool valid = false;
   	Eigen::Vector3d startK;
	Eigen::Vector3d endK;

};
} /* namespace A2O */

