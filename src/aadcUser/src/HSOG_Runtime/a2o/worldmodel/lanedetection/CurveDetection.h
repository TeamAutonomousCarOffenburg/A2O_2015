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

class CurveDetection : public Detection
{
public:
	float radius;
	Eigen::Vector3d middleK;
	Eigen::Vector3d centerK;

	cv::Point start;
	cv::Point middle;
	cv::Point end;

	/**
	 * Returns true if the end is left of the start.
	 * Does not check validity!
	 */
	bool isLeft()
	{
		if (centerK(1) > startK(1)) {
			return true;
		}
		return false;
	}

	/**
	 * Returns the distanc of the start to end point (camera coordinates)
	 */
	float getCurveLength()
	{
		return (endK - startK).norm();
	}

	/**
	 * Returns the distanc of the start to end point (pixel coordinates)
	 */
	float getCurvePixelLength()
	{
		return Line(start, end).getLength();
	}

	float calculateCurveRadius() {
		radius = -1;

		if (!valid) {
			return radius;
		}
		/*
		 a = sqrt((x1-x2)^2+(y1-y2)^2); % The three sides
		 b = sqrt((x2-x3)^2+(y2-y3)^2);
		 c = sqrt((x3-x1)^2+(y3-y1)^2);
		 s = (a+b+c)/2;
		 A = sqrt(s*(s-a)*(s-b)*(s-c)); % Area of triangle
		 R = a*b*c/(4*A); % Radius of circumscribing circle
		 */

		float dx1 = startK(0) - middleK(0);
		float dy1 = startK(1) - middleK(1);
		float a = sqrt(dx1 * dx1 + dy1 * dy1);

		float dx2 = middleK(0) - endK(0);
		float dy2 = middleK(1) - endK(1);
		float b = sqrt(dx2 * dx2 + dy2 * dy2);

		float dx3 = endK(0) - startK(0);
		float dy3 = endK(1) - startK(1);
		float c = sqrt(dx3 * dx3 + dy3 * dy3);

		float s = (a + b + c) * 0.5;
		float root = s * (s - a) * (s - b) * (s - c);
		if (root <= 0) {
			// points are on same line
			return radius;
		}
		float A = sqrt(root);
		radius = a * b * c / (4 * A);

		//std::cerr<<"R : " << r <<std::endl;
		return radius;
	}

	void calculateAlternativeMiddle()
	{
		if (!valid) {
			return;
		}
		float x1 = startK(0);
		float x2 = middleK(0);
		float x3 = endK(0);
		float y1 = startK(1);
		float y2 = middleK(1);
		float y3 = endK(1);

		
		float ma,mb,cx,ya,yb,cy,numerator,denominator;
		
		ma = (y2-y1)/(x2-x1);
		mb = (y3-y2)/(x3-x2);
		
		denominator = 2* (mb -ma);
		numerator = (ma*mb*(y1-y3)) + (mb*(x1+x2)) +(ma*(x2+x3));
		
		cx = (numerator)/(denominator);	
		
		if (denominator > -0.000001 && denominator < 0.000001) {
			// avoid division by zero (points on one line)
			valid = false;
			return;
		}
	
		if ((ma > -0.000001 && ma < 0.000001)||(mb > -0.000001 && mb < 0.000001)) {
			// avoid division by zero (points on one line)
			valid = false;
			return;
		}
	
		
		ya =  (-1/ma)*(cx-((x1+x2)/2))+(y1+y2)/2;
		yb =  (-1/mb)*(cx-((x2+x3)/2))+(y2+y3)/2;
		
		cy = (ya+yb)/2;
		
		// std::cout << "x: " << xm << " y: " << ym << std::endl;

		centerK = Eigen::Vector3d(cx, cy, 0);
		float radius1 = (centerK - startK).norm();
		float radius2 = (centerK - middleK).norm();
		float radius3 = (centerK - endK).norm();
		radius = (radius1 + radius2 + radius3) / 3;
	}
	
	
	void calculateMiddle()
	{
		if (!valid) {
			return;
		}
		float x1 = startK(0);
		float x2 = middleK(0);
		float x3 = endK(0);
		float y1 = startK(1);
		float y2 = middleK(1);
		float y3 = endK(1);

		// ym = [ (x32 - x12 + y32 - y12)(x2 - x1) - (x22 - x12 + y22 - y12)(x3 - x1)]
		// /  2[(y3 - y1)(x2 - x1) - (y2 - y1)(x3 - x1)]

		// xm = [ (x22 - x12) + (y22 - y12) - 2ym(y2 - y1) ] / 2(x2 - x1)

		float nenner = 2 * ((y3-y1) * (x2 - x1) - (y2 - y1) * (x2 - x1));
		if (nenner > -0.000001 && nenner < 0.000001) {
			// avoid division by zero (points on one line)
			valid = false;
			return;
		}
		float nenner2 = 2 * (x2 - x1);
		if (nenner2 > -0.000001 && nenner2 < 0.000001) {
			// avoid division by zero (points on one line)
			valid = false;
			return;
		}

		float ym = ((x3*x3 - x1*x1 + y3*y3 - y1*y1) * (x2-x1) - (x2*x2 - x1*x1 + y2*y2 - y1*y1) * (x3 - x1)) / nenner;
		float xm = ((x2*x2 - x1*x1) + (y2*y2 - y1*y1) - 2 * ym * (y2 - y1)) / nenner2;

		// std::cout << "x: " << xm << " y: " << ym << std::endl;

		centerK = Eigen::Vector3d(xm, ym, 0);
		float radius1 = (centerK - startK).norm();
		float radius2 = (centerK - middleK).norm();
		float radius3 = (centerK - endK).norm();
		radius = (radius1 + radius2 + radius3) / 3;
	}

};
} /* namespace A2O */

