#pragma once
/*
* MathUtils.h
* Functions for custom Math Operations
*  Created on: Aug 17, 2018
*      Author: Aaron Boda
*/

#ifndef MATHUTILS_H_
#define MATHUTILS_H_

#include <cmath>

// Functions
double deg2rad(double ang);
double rad2deg(double ang);
double sec(double ang);
double normalise(const double value, const double start, const double end);

#endif /* MATHUTILS_H_ */
#pragma once
