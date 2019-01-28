/*
* MathUtils.cpp
* Functions for custom Math Operations
*  Created on: Aug 17, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "MathUtils.h"
using namespace std;

const double PI = 3.1415926535898;

// degrees to radians
double rad2deg(double ang) {
	double deg = ang * (180 / PI);
	return deg;
}

// radians to degrees
double deg2rad(double ang) {
	double rad = ang * (PI / 180);
	return rad;
}

// missing secant function 
double sec(double ang) {
	double ans = 1 / cos(ang);
	return ans;
}

//Normalizes any number to an arbitrary range 
//by assuming the range wraps around when going below min or above max 
double normalise(const double value, const double start, const double end)
{
	const double width = end - start;   // 
	const double offsetValue = value - start;   // value relative to 0

	return (offsetValue - (floor(offsetValue / width) * width)) + start;
	// + start to reset back to start of original range
}