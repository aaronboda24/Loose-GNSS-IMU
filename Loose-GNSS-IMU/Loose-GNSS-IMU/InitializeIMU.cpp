/*
* InitializeIMU.cpp
* Helps with initializing IMU states
*  Created on: Aug 26, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "InitializeIMU.h"
using namespace std;
using namespace Eigen;

// Constants
const double Om = 7.2921155e-5;
const double PI = 3.1415926536;

// Gravitational acceleration ECEF components
VectorXd gravityECEF(double lat, double height, MatrixXd Cne) {
	double WIE_E = 7292115e-11;
	double SM_AXIS = 6378137;
	double E_SQR = 0.00669437999014;
	double NORMAL_GRV = 9.7803253359;
	double GRV_CONS = 0.00193185265241;
	double FLATTENING = 0.00335281066475;
	double M_FAKTOR = 0.00344978650684;
	double sL = sin(lat);
	double cL = cos(lat);
	double h = height;
	double Rn = 6335439.327292829 / (sqrt(1.0 - E_SQR*sL*sL)*(1.0 - E_SQR*sL*sL));
	double Re = SM_AXIS / (sqrt(1.0 - E_SQR*sL*sL));
	double g1 = NORMAL_GRV*(1 + GRV_CONS*sL*sL) / (sqrt(1.0 - E_SQR*sL*sL));
	double g = g1*(1.0 - (2.0 / SM_AXIS)*(1.0 + FLATTENING + M_FAKTOR - 2.0*FLATTENING*sL*sL)*h + 3.0*h*h / SM_AXIS / SM_AXIS);
	// ECEF gravity
	VectorXd grav(3); grav.setZero();
	grav(0) = Cne(0, 2) * g;
	grav(1) = Cne(1, 2) * g;
	grav(2) = Cne(2, 2) * g;
	return grav;
}
	
// Constructor : Static Initialization of IMU Attitude
InitializeIMU::InitializeIMU(ifstream& fin_imu, double EndTime, vector<double> LLH) {
	ReaderIMU OBSimu; OBSimu.obsEpoch(fin_imu);
	// Running average variables
	double Ax_avg = 0; double Ay_avg = 0; double Az_avg = 0;
	double Gx_avg = 0; double Gy_avg = 0; double Gz_avg = 0;
	int count = 0;
	// Iterate through IMU file
	while ((!fin_imu.eof()) && (OBSimu._IMUdata.imuTime < EndTime)) {
		count++;
		// Reading IMU Epoch
		OBSimu.clearObs();
		OBSimu.obsEpoch(fin_imu);
		// Computing running sum
		Ax_avg = Ax_avg + OBSimu._IMUdata.Ax;
		Ay_avg = Ay_avg + OBSimu._IMUdata.Ay;
		Az_avg = Az_avg + OBSimu._IMUdata.Az;
		Gx_avg = Gx_avg + OBSimu._IMUdata.Gx;
		Gy_avg = Gy_avg + OBSimu._IMUdata.Gy;
		Gz_avg = Gz_avg + OBSimu._IMUdata.Gz;
	}
	// Compute rough estimate of Yaw
	double numer = -Gx_avg / count; double denom = Gy_avg / count;
	double yaw = atan(numer / denom);
	// Gravity components ECEF
	MatrixXd Cbe(3, 3); Cbe.setZero(); Cbe = b2eDCM(LLH.at(0), LLH.at(1), 0, 0, yaw);
	MatrixXd Cne(3, 3); Cne.setZero(); Cne = llf2ecefDCM(LLH.at(0), LLH.at(1));
	VectorXd grav(3); grav.setZero(); grav = gravityECEF(LLH.at(0), LLH.at(2), Cne);
	// Compute Accelerometer Bias
	VectorXd fbib(3); fbib.setZero();
	fbib(0) = Ax_avg / count; fbib(1) = Ay_avg / count; fbib(2) = Az_avg / count;
	VectorXd feib(3); feib.setZero(); feib = Cbe * fbib;
	VectorXd fbias_e(3); fbias_e.setZero(); fbias_e = feib + grav;
	VectorXd fbias_b(3); fbias_b.setZero(); fbias_b = Cbe.transpose() * fbias_e;
	_ACCbias = eigVector2std(fbias_b);
	// Compute Gyroscope Bias
	MatrixXd Cbn(3, 3); Cbn.setZero(); Cbn = b2llfDCM(0, 0, yaw);
	VectorXd Om_b_ib(3); Om_b_ib.setZero(); Om_b_ib(0) = Gx_avg / count; Om_b_ib(1) = Gy_avg / count; Om_b_ib(2) = Gz_avg / count;
	VectorXd Om_b_nb(3); Om_b_nb.setZero(); Om_b_nb = std2eigVector(Transformer(eigVector2std(Om_b_ib), Cbn));
	VectorXd Om_n_ie(3); Om_n_ie.setZero(); Om_n_ie(1) = Om * cos(LLH.at(0)); Om_n_ie(2) = Om * sin(LLH.at(0));
	VectorXd gbias_n(3); gbias_n.setZero(); gbias_n = Om_b_nb - Om_n_ie;
	VectorXd gbias_b(3); gbias_b.setZero(); gbias_b = std2eigVector(Transformer(eigVector2std(gbias_n), Cbn.transpose()));
	_GYRbias = eigVector2std(gbias_b);

	// Computing Average
	Ax_avg = (Ax_avg / count) - fbias_b(0);
	Ay_avg = (Ay_avg / count) - fbias_b(1);
	Az_avg = (Az_avg / count) - fbias_b(2);
	Gx_avg = (Gx_avg / count) - gbias_b(0);
	Gy_avg = (Gy_avg / count) - gbias_b(1);
	Gz_avg = (Gz_avg / count) - gbias_b(2);

	// Add average imu measurements to vector
	_GYRavg.push_back(Gx_avg); _GYRavg.push_back(Gy_avg); _GYRavg.push_back(Gz_avg);
	_ACCavg.push_back(Ax_avg); _ACCavg.push_back(Ay_avg); _ACCavg.push_back(Az_avg);

	// Rotate measurements
	Gx_avg = _GYRavg.at(0); Gy_avg = _GYRavg.at(1); Gz_avg = _GYRavg.at(2);
	Ax_avg = _ACCavg.at(0); Ay_avg = _ACCavg.at(1); Az_avg = _ACCavg.at(2);

	// *** As per Paul Groves' Textbook
	// Compute Roll and Pitch
	_roll = atan2(-Ay_avg, Az_avg);
	_pitch = atan(-Ax_avg / (sqrt(pow(Ay_avg, 2) + pow(Az_avg, 2))));
	// Compute Yaw
	double num = -Gy_avg * cos(_roll) + Gz_avg * sin(_roll);
	double den = Gx_avg * cos(_pitch) + Gy_avg * sin(_pitch) * sin(_roll) + Gz_avg * cos(_roll) * sin(_pitch);
	_yaw = atan2(num, den);
	// Add roll pitch yaw to vector
	_RPY.push_back(_roll); _RPY.push_back(_pitch); _RPY.push_back(_yaw);
}

// Destructor
InitializeIMU::~InitializeIMU() {
	_GYRavg.clear(); _ACCavg.clear(); _RPY.clear(); _GYRbias.clear(); _ACCbias.clear();
	_roll = NULL; _pitch = NULL; _yaw = NULL;
}
