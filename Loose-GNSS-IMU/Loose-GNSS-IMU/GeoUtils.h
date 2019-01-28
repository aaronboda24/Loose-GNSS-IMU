/*
* GeoUtils.h
* Provides options and utilities for geodetic calculations
*  Created on: Jun 18, 2017
*      Author: Aaron Boda
*/

#ifndef GEOUTILS_H_
#define GEOUTILS_H_

#include "pch.h"

// Functions
void radius(double &_Rm, double &_Rn, double lat);
std::vector<double> earthrate(double lat);
std::vector<double> ecef2geo(std::vector<double> ecefXYZ);
std::vector<double> enu2ecef(std::vector<double> ENU_i, std::vector<double> ECEF_o, std::vector<double> LLH_o);
std::vector<double> ecef2enu(std::vector<double> ECEF_i, std::vector<double> ECEF_o, std::vector<double> LLH_o);
std::vector<double> transrate(double lat, double _Rm, double _Rn, double vn, double ve, double h);

#endif /* GEOUTILS_H_ */


