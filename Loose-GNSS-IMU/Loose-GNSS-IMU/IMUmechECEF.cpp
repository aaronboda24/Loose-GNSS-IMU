/*
* IMUmechECEF.cpp
* ECEF Mechanization of IMU
*  Created on: Sept 10, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "IMUmechECEF.h"
using namespace std;
using namespace Eigen;

// Constants
const double Om = 7.2921155e-5;
const double PI = 3.1415926535898;
const double mu = 3.986005e14; // Earths Gravitational Constant
const double J2 = 1.082630e-3; // Second Gravitational Constant
const double re = 6378137;	 // Equatorial radius
							
// Constructor
IMUmechECEF::IMUmechECEF(){}

// Destructor
IMUmechECEF::~IMUmechECEF() {
	_att.clear();
	_pos.clear();
	_vel.clear();
}

// Computes heading correction for Gyro
double IMUmechECEF::HeadingCorrection(double imuHeading, double gpsHeading) {
	double yawDelta = imuHeading - gpsHeading;
	if (yawDelta < 0) { yawDelta = yawDelta + (2 * PI); }
	if ((yawDelta >= (-PI / 2)) && (yawDelta < (PI / 2))) { yawDelta *= -1.0; }
	else {
		if (yawDelta >(PI / 2)) { yawDelta = (2 * PI) - yawDelta; }
		else { yawDelta = (2 * PI + yawDelta) * -1.0; }
	}
	if (yawDelta >(2 * PI)) { yawDelta -= 2 * PI; }
	if (yawDelta < (-2 * PI)) { yawDelta += 2 * PI; }
	return yawDelta;
}

// Initializer
void IMUmechECEF::InitializeMechECEF(vector<double> iniPOS, vector<double> iniLLH, vector<double> iniVEL, 
	vector<double> iniATT, vector<double> Fbias, vector<double> Gbias) {
	// Initializing the struct
	_att = iniATT;
	_pos = iniPOS;
	_vel = iniVEL;
	_LLHo = iniLLH;
	_ECEFo = iniPOS;
	_Fe = MatrixXd::Zero(3, 3);
	_Ne = MatrixXd::Zero(3, 3);
	_Cbe = MatrixXd::Zero(3, 3);
	_Cbn = MatrixXd::Zero(3, 3);
	_Cbe = b2eDCM(iniLLH.at(0), iniLLH.at(1), _att.at(0), _att.at(1), _att.at(2));
	_Obib = MatrixXd::Zero(3, 3);
	_Oeie = MatrixXd::Zero(3, 3);
	_Heading_diff = 0;
	// Initial Bias Values
	_fbias = Fbias; _gbias = Gbias;
	// Lever Arm
	double LAx = -0.964, LAy = -0.924, LAz = -0.196;
	_Lxyz = VectorXd::Zero(3);
	_Lxyz(0) = LAx; _Lxyz(1) = LAy; _Lxyz(2) = LAz;
}

// Computes gravitational acceleration ECEF components
VectorXd IMUmechECEF::gravityECEF(double X, double Y, double Z) {
	// Variables
	const double RR = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
	const double R = sqrt(RR);
	const double aoR = pow((6378137.0 / R) , 2);
	const double roR = pow((re / R), 2);
	const double ZoR = pow(sin(Z / R), 2);
	// Gravitational Acceleration in Earth Ceneterd Coordinates
	double Gx = (-mu / (RR)) * (1 + (3 / 2) * (aoR) * J2 * (roR * (1 - 5 * (ZoR)))) * (X / R);
	double Gy = (-mu / (RR)) * (1 + (3 / 2) * (aoR) * J2 * (roR * (1 - 5 * (ZoR)))) * (Y / R);
	double Gz = (-mu / (RR)) * (1 + (3 / 2) * (aoR) * J2 * (roR * (3 - 5 * (ZoR)))) * (Z / R);
	// ECEF Gravity Vector
	VectorXd r = VectorXd::Zero(3); r(0) = X; r(1) = Y; r(2) = Z;
	VectorXd G = VectorXd::Zero(3); G(0) = Gx; G(1) = Gy; G(2) = Gz;
	MatrixXd Oie = MatrixXd::Zero(3, 3); Oie(0, 1) = -Om; Oie(1, 0) = Om;
	VectorXd g = VectorXd::Zero(3); g = G - Oie * (Oie * r);
	return g;
}

// Tensor of Gravity Gradients 
MatrixXd IMUmechECEF::TensorGravGrad(double X, double Y, double Z) {
	MatrixXd Ne = MatrixXd::Zero(3, 3);
	// Variables
	const double R = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));
	const double R2 = pow(R, 2);
	const double R3 = pow(R, 3);
	const double M = 5.972e24;
	const double G = 6.674e-11;
	// Elements of Ne
	Ne(0, 0) = (G*M / R3)*((3 * X*X / R2) - 1) + pow(Om, 2);
	Ne(0, 1) = (G*M / R3)*(3 * X*Y / R2);
	Ne(0, 2) = (G*M / R3)*(3 * X*Z / R2);
	Ne(1, 0) = (G*M / R3)*(3 * X*Y / R2);
	Ne(1, 1) = (G*M / R3)*((3 * Y*Y / R2) - 1) + pow(Om, 2);
	Ne(1, 2) = (G*M / R3)*(3 * Y*Z / R2);
	Ne(2, 0) = (G*M / R3)*(3 * X*Z / R2);
	Ne(2, 1) = (G*M / R3)*(3 * Y*Z / R2);
	Ne(2, 2) = (G*M / R3)*((3 * Z*Z / R2) - 1) + pow(Om, 2);
	return Ne;
}

// ECEF Mechanization of IMU
void IMUmechECEF::MechanizerECEF(double dT, vector<double> acc, vector<double> gyr, vector<double> LLH) {
	// Observation Arrangement
	vector<double> fib;
	fib.push_back(acc.at(0) - _fbias.at(0)); fib.push_back(acc.at(1) - _fbias.at(1)); fib.push_back(acc.at(2) - _fbias.at(2));
	vector<double> wib;
	wib.push_back(gyr.at(0) - _gbias.at(0)); wib.push_back(gyr.at(1) - _gbias.at(1)); wib.push_back(gyr.at(2) - _gbias.at(2));

	// --- Constants ---
	VectorXd om_eie = VectorXd::Zero(3); om_eie(2) = Om;

	// --- Initializing ---
	VectorXd Pos0 = VectorXd::Zero(3); Pos0 = std2eigVector(_pos);
	VectorXd Vel0 = VectorXd::Zero(3); Vel0 = std2eigVector(_vel);
	VectorXd Att0 = VectorXd::Zero(3); Att0 = std2eigVector(_att);
	MatrixXd Cbe0 = MatrixXd::Zero(3, 3); Cbe0 = _Cbe;

	// --- Vectorizing Measurements ---
	VectorXd om_bib = VectorXd::Zero(3); om_bib = std2eigVector(wib);
	VectorXd sf_bib = VectorXd::Zero(3); sf_bib = std2eigVector(fib);
	_Obib = SkewMat(om_bib);
	_Oeie = SkewMat(om_eie);

	// --- Attitude Update ---
	MatrixXd Cbe = MatrixXd::Zero(3, 3);
	Cbe = Cbe0 * (MatrixXd::Identity(3, 3) + (SkewMat(om_bib) * dT)) - SkewMat(om_eie) * Cbe0 * dT;

	// --- Specific Force Transformation ---
	VectorXd sf_eib = VectorXd::Zero(3);
	sf_eib = (0.5 * (Cbe0 + Cbe)) * sf_bib;

	// --- Gravity Vector in ECEF ---
	VectorXd g_eb = VectorXd::Zero(3); 
	g_eb = gravityECEF(Pos0(0), Pos0(1), Pos0(2));

	// --- Velocity Update ---
	VectorXd vel_eeb = VectorXd::Zero(3);
	vel_eeb = Vel0 + (sf_eib - g_eb - 2 * SkewMat(om_eie) * Vel0) * dT;

	//cout << sf_eib.transpose() << "\n" << g_eb.transpose() << "\n\n";

	// --- Position Update ---
	VectorXd pos_eeb = VectorXd::Zero(3);
	pos_eeb = Pos0 + (Vel0 + vel_eeb) * 0.5 * dT;

	// --- Compute Roll Pitch Yaw ---
	MatrixXd Cen = MatrixXd::Zero(3, 3);
	Cen = ecef2llfDCM(LLH.at(0), LLH.at(1));
	_Cbn = MatrixXd::Zero(3, 3);
	_Cbn = Cen * Cbe;
	vector<double> euler; euler = dcm2euler(_Cbn.transpose());
	double roll = euler.at(0);
	double pitch = euler.at(1);
	double yaw = euler.at(2);
	Att0(0) = roll; Att0(1) = pitch; Att0(2) = yaw;
	NormaliseAttitudeOnly(Att0);

	// --- Update ---
	// Attitude
	_att.clear();
	_att = eigVector2std(Att0);
	// Position
	_pos.clear();
	_pos = eigVector2std(pos_eeb);
	// Velocity
	_vel.clear();
	_vel = eigVector2std(vel_eeb);
	// Body to Earth DCM
	_Cbe = Cbe;
	// Specific Force in earth frame
	_Fe = SkewMat(sf_eib);
	// Tensor Gravity Gradient
	_Ne = TensorGravGrad(_pos.at(0), _pos.at(1), _pos.at(2));
}
 