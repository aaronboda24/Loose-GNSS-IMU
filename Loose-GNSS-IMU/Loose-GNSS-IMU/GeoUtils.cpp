/*
* GeoUtils.cpp
* Provides options and utilities for geodetic calculations
*  Created on: Jun 18, 2017
*      Author: Aaron Boda
*/

#include "pch.h"
#include "GeoUtils.h"
using namespace std;

// Constants
const double PI = 3.1415926535898;
const double Om = 7.2921155e-5;
const double ECC = 0.0818191908426;
const double SEMI_MAJOR = 6378137.0;

// Coverts from ECEF to Geodetic Coordinates
vector<double> ecef2geo(vector<double> ecefXYZ) {
	// Output vector - Lat, Long, Height
	vector<double> GEO;
	// Variables
	double x, y, z;
	x = ecefXYZ.at(0); y = ecefXYZ.at(1); z = ecefXYZ.at(2);
	// Semi Major Axis and Eccentricity
	const double a = 6378137; const double e = 0.08181979;
	// Compute Longitude
	double lambda = atan2(y, x);
	// Physical radius of the point 
	double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	// Radius in the x-y plane 
	double p = sqrt(pow(x, 2) + pow(y, 2));
	// GEOcentric latitude (Initial Approx)
	double phi_o = atan2(p, z); double phi_i = phi_o;
	// Radius of curvature in the prime vertical
	double Rn;
	// Height
	double h;
	// Loop
	for (unsigned i = 0; i < 3; i++) {
		// Recalculate Radius of curvature in the prime vertical
		Rn = a / sqrt(1 - (e * e * sin(phi_i) * sin(phi_i)));
		// Recalculate Height
		h = (p / cos(phi_i)) - (Rn);
		// Recalculate Latitude
		phi_i = atan((z / p) * (pow((1 - ((pow(e, 2))*(Rn / (Rn + h)))), (-1))));
	}
	// Recalculate Height
	h = (p / cos(phi_i)) - Rn;
	// Populate output vector
	GEO.push_back(phi_i);
	GEO.push_back(lambda);
	GEO.push_back(h);
	return GEO;
}

// Coverts from ECEF to ENU Local System
vector<double> ecef2enu(vector<double> ECEF_i, vector<double> ECEF_o, vector<double> LLH_o) {
	// Output vector
	vector<double> ENU;
	// Variables
	double dX = ECEF_i.at(0) - ECEF_o.at(0);
	double dY = ECEF_i.at(1) - ECEF_o.at(1);
	double dZ = ECEF_i.at(2) - ECEF_o.at(2);
	double lat = LLH_o.at(0); double lon = LLH_o.at(1);	
	// Compute ENU
	double E = -sin(lon) * dX + cos(lon) * dY;
	double N = -sin(lat) * cos(lon) * dX - sin(lat) * sin(lon) * dY + cos(lat) * dZ;
	double U = cos(lat) * cos(lon) * dX + cos(lat) * sin(lon) * dY + sin(lat) * dZ;
	// Add to vector and output
	ENU.push_back(E);
	ENU.push_back(N);
	ENU.push_back(U);
	return ENU;
}

// Coverts from ENU to ECEF Local System
vector<double> enu2ecef(vector<double> ENU, vector<double> ECEF_o, vector<double> LLH_o) {
	// Variables
	double E = ENU.at(0);
	double N = ENU.at(1);
	double U = ENU.at(2);
	double lat = LLH_o.at(0); double lon = LLH_o.at(1);
	// Output vector
	double dX = -sin(lon)*E - sin(lat)*cos(lon)*N + cos(lat)*cos(lon)*U;
	double dY =  cos(lon)*E - sin(lat)*sin(lon)*N + cos(lat)*sin(lon)*U;
	double dZ =  cos(lat)*N + sin(lat)*U;
	// Add deltas to ECEFo
	vector<double> XYZ;
	XYZ.push_back(ECEF_o.at(0) + dX);
	XYZ.push_back(ECEF_o.at(1) + dY);
	XYZ.push_back(ECEF_o.at(2) + dZ);
	return XYZ;
}


// Updates Rm and Rn
// lat = latitude in degrees
void radius(double &_Rm, double &_Rn, double lat) {
	lat = lat;
	double e2 = ECC * ECC;
	double den = 1 - e2 * pow((sin(lat)), 2);
	// Meridian radius of curvature: radius of curvature for north-south motion
	_Rm = SEMI_MAJOR * ((1 - e2) / pow((den), 1.5));
	// Normal radius of curvature: radius of curvature for east-west motion
	_Rn = SEMI_MAJOR / sqrt(den);
}

// Computes turn rate of the Earth in the navigation frame
vector<double> earthrate(double lat) {
	lat = lat;
	vector<double> Om_ie_n;
	Om_ie_n.push_back(Om * cos(lat));
	Om_ie_n.push_back(0);
	Om_ie_n.push_back(Om * -sin(lat));
	return Om_ie_n;
}

// Computes the transport rate in the navigation frame
// lat = latitude, Vn = velocity in N, Ve = velocity in E, h = altitude
vector<double> transrate(double lat, double _Rn, double _Rm, double Vn, double Ve, double h) {
	vector<double> Om_en_n;
	Om_en_n.push_back((Ve / (_Rn + h)));
	Om_en_n.push_back((-(Vn / (_Rm + h))));          
	Om_en_n.push_back((-(Ve * tan(lat) / (_Rn + h))));
	return Om_en_n;
}