/*
* DCM.cpp
* Direction Cosine Matrices
*  Created on: Aug 07, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "DCM.h"
using namespace std;
using namespace Eigen;

// PI
const double PI = 3.1415926535898;

// ECEF to LLF DCM -> lat, lon in radians
MatrixXd ecef2llfDCM(double lat, double lon) {
	MatrixXd Cen = MatrixXd::Zero(3, 3);
	Cen(0, 0) = -cos(lon)*sin(lat);	Cen(0, 1) = -sin(lat)*sin(lon);	Cen(0, 2) = cos(lat);
	Cen(1, 0) = -sin(lon);			Cen(1, 1) = cos(lon);			Cen(1, 2) = 0;
	Cen(2, 0) = -cos(lat)*cos(lon);	Cen(2, 1) = -cos(lat)*sin(lon);	Cen(2, 2) = -sin(lat);
	return Cen;
}

// LLF to ECEF DCM -> lat, lon in radians
MatrixXd llf2ecefDCM(double lat, double lon) {
	MatrixXd Cen = MatrixXd::Zero(3, 3);
	MatrixXd Cne = MatrixXd::Zero(3, 3);
	Cen = ecef2llfDCM(lat, lon);
	Cne = Cen.transpose();
	return Cne;
}

// LLF to Body (B1) DCM -> r=roll, p=pitch, y=yaw
MatrixXd llf2bDCM(double r, double p, double y) {
	MatrixXd Cnb = MatrixXd::Zero(3, 3);
	Cnb = (Rx(r) * Ry(p)) * Rz(y);
	return Cnb;
}

// Body (B1) to LLF DCM -> r=roll, p=pitch, y=yaw
MatrixXd b2llfDCM(double r, double p, double y) {
	MatrixXd Cbn = MatrixXd::Zero(3, 3);
	MatrixXd Cnb = MatrixXd::Zero(3, 3);
	Cnb = llf2bDCM(r, p, y);
	Cbn = Cnb.transpose();
	return Cbn;
}

// Body (B1) to ECEF DCM -> lat = latitude, lon = longitude, r=roll, p=pitch, y=yaw
MatrixXd b2eDCM(double lat, double lon, double r, double p, double y) {
	MatrixXd n2e = MatrixXd::Zero(3, 3); n2e = llf2ecefDCM(lat, lon);
	MatrixXd b2n = MatrixXd::Zero(3, 3); b2n = b2llfDCM(r, p, y);
	MatrixXd b2e = MatrixXd::Zero(3, 3); b2e = n2e * b2n;
	return b2e;
}

// ECEF to Body (B1) DCM -> lat = latitude, lon = longitude, r=roll, p=pitch, y=yaw
MatrixXd e2bDCM(double lat, double lon, double r, double p, double y) {
	MatrixXd b2e = MatrixXd::Zero(3, 3); b2e = b2eDCM(lat, lon, r, p, y);
	MatrixXd e2b = MatrixXd::Zero(3, 3); e2b = b2e.transpose();
	return e2b;
}

// Find roll pitch yaw in radians, given enu2b1 DCM
vector<double> dcm2euler(Eigen::MatrixXd Cnb) {
	double roll = atan(Cnb(1, 2) / Cnb(2, 2));
	double pitch = asin(-Cnb(0, 2));
	double yaw = atan(Cnb(0, 1) / Cnb(0, 0));
	vector<double> rpy;
	rpy.push_back(roll); rpy.push_back(pitch); rpy.push_back(yaw);
	return rpy;
}
