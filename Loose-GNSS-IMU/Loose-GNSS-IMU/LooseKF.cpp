/*
* LooseKF.h
* 21 state Kalman Filter Data Holder for Loosely Coupled Integration
*  Created on: Sept 12, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "LooseKF.h"
using namespace Eigen;
using namespace std;

const double Om = 7.2921155e-5;
const double PI = 3.1415926535898;
const int movingWindow = 50;

// Develop transition matrix
void LooseKF::Transition(double dT, MatrixXd Ne, MatrixXd Fe, MatrixXd Cbe) {
	_F.setZero();
	MatrixXd dfdx = MatrixXd::Zero(3, 3);
	VectorXd Oeie = VectorXd::Zero(3); Oeie(2) = Om;

	// --- Derivatives wrt position model ---
	// 1) for position
	dfdx.setIdentity();
	_F.block<3, 3>(0, 0) = dfdx;
	// 2) for velocity
	dfdx.setIdentity();
	dfdx = dfdx * dT;
	_F.block<3, 3>(0, 3) = dfdx;
	
	// --- Derivatives wrt velocity model ---
	// 1) for position
	_F.block<3, 3>(3, 0) = Ne;
	// 2) for velocity
	dfdx.setZero();
	dfdx = MatrixXd::Identity(3, 3) - 2 * dT * SkewMat(Oeie);
	_F.block<3, 3>(3, 3) = dfdx;
	// 3) for attitude
	dfdx.setZero();
	dfdx = -dT * Fe;
	_F.block<3, 3>(3, 6) = dfdx;
	// 5) for specific force
	dfdx.setZero();
	dfdx = dT * Cbe;
	_F.block<3, 3>(3, 12) = dfdx;

	// --- Derivatives wrt Attitude model ---
	// 3) for attitude
	dfdx.setZero();
	dfdx = MatrixXd::Identity(3, 3) - dT * SkewMat(Oeie);
	_F.block<3, 3>(6, 6) = dfdx;
	// 4) for angular rate
	dfdx.setZero();
	dfdx = dT * Cbe;
	_F.block<3, 3>(6, 9) = dfdx;

	// --- Derivatives wrt Angular Rate Model ---
	// 4) for angular rate
	MatrixXd Dg(3, 3); Dg.setConstant(0.0007);
	dfdx.setZero(); 
	dfdx = MatrixXd::Identity(3, 3) + dT * Dg;
	_F.block<3, 3>(9, 9) = dfdx;

	// --- Derivatives wrt Specific Force model ---
	// 5) for specific force
	MatrixXd Da(3, 3); Da.setConstant(0.003);
	dfdx.setZero();
	dfdx = MatrixXd::Identity(3, 3) + dT * Da;
	_F.block<3, 3>(12, 12) = dfdx;
}

// Develop process noise coefficient matrix
void LooseKF::ProcessNoiseCoeff(double dT, MatrixXd Cbe) {
	_G.setZero();
	_G.block<3, 3>(9, 0)  = Cbe;
	_G.block<3, 3>(12, 3) = Cbe;
	// Compute Process Nosie Covariance
	_Q = (_F * _G * _Qw * _G.transpose() * _F.transpose() + _G * _Qw * _G.transpose()) * dT * 0.5;
}

// Sets observation vector of IMU
void LooseKF::SetObs(ReaderGNSS &GNSS, IMUmechECEF& IMU) {
	// Lever Arm Calculation
	VectorXd dL_pos_ecef(3); dL_pos_ecef.setZero();
	dL_pos_ecef = IMU._Cbe * IMU._Lxyz;
	VectorXd dL_vel_ecef(3); dL_vel_ecef.setZero();
	dL_vel_ecef = IMU._Oeie * IMU._Cbe * IMU._Lxyz + IMU._Cbe * IMU._Obib * IMU._Lxyz;

	// Arranging observation vector
	_Zobs = VectorXd::Zero(6);
	_Robs = MatrixXd::Zero(6, 6);
	for (int i = 0; i < 3; i++) {
		// Observations
		_Zobs(i) = IMU._pos[i] - GNSS._GNSSdata.Pxyz(i);
		_Zobs(i + 3) = IMU._vel[i] - GNSS._GNSSdata.Vxyz(i);
	}	
	// Variance
	_Robs.topLeftCorner(3, 3) = GNSS._GNSSdata.CovPxyz;
	_Robs.bottomRightCorner(3, 3) = GNSS._GNSSdata.CovVxyz;
	// Covariance
	_Robs.topRightCorner(3, 3) = (GNSS._GNSSdata.CovPVxyz);
	_Robs.bottomLeftCorner(3, 3) = (GNSS._GNSSdata.CovPVxyz.transpose());

}

// Kalman Filter Algorithm
void LooseKF::Filter(IMUmechECEF &IMU) {
	VectorXd I = VectorXd::Zero(6);
	MatrixXd K = MatrixXd::Zero(15, 6);
	MatrixXd H = MatrixXd::Identity(6, 15);

	// State Covariance Prediction
	_Ppre = _F * _Pupd * _F.transpose() + _Q;
	// State Prediction
	_Xpre = _F * _Xupd;
	// Compute Innovation
	I = _Zobs;
	// Gain
	K = _Ppre * H.transpose() * ((H * _Ppre * H.transpose() + _Robs).inverse());
	// State Estimate (Updated)
	_Xupd = _Xpre + (K * I);
	// State Covaraince (Updated)
	_Pupd = (MatrixXd::Identity(15, 15) - K * H) * _Ppre;

	// Update States
	sol.posXYZ.clear();
	sol.posXYZ.push_back(IMU._pos.at(0) - _Xupd(0)); 
	sol.posXYZ.push_back(IMU._pos.at(1) - _Xupd(1));
	sol.posXYZ.push_back(IMU._pos.at(2) - _Xupd(2));
	sol.velXYZ.push_back(IMU._vel.at(0) - _Xupd(3));
	sol.velXYZ.push_back(IMU._vel.at(1) - _Xupd(4));
	sol.velXYZ.push_back(IMU._vel.at(2) - _Xupd(5));
	sol.attXYZ.push_back(IMU._att.at(0) - _Xupd(6));
	sol.attXYZ.push_back(IMU._att.at(1) - _Xupd(7));
	sol.attXYZ.push_back(IMU._att.at(2) - _Xupd(8));
	sol.dw.push_back(IMU._gbias.at(0) - _Xupd(9)); 
	sol.dw.push_back(IMU._gbias.at(1) - _Xupd(10));
	sol.dw.push_back(IMU._gbias.at(2) - _Xupd(11));
	sol.df.push_back(IMU._fbias.at(0) - _Xupd(12));
	sol.df.push_back(IMU._fbias.at(1) - _Xupd(13));
	sol.df.push_back(IMU._fbias.at(2) - _Xupd(14));
}

// CONSTRUCTOR AND DESTRUCTOR DEFINITIONS
LooseKF::LooseKF(IMUmechECEF &IMU, double dT) {
	// Number of states
	size_t u = 15;
	// Initialize matrices
	_Xpre = VectorXd::Zero(u);
	_Xupd = VectorXd::Zero(u);
	_F = MatrixXd::Zero(u, u);
	_Q = MatrixXd::Zero(u, u);
	_Ppre = MatrixXd::Zero(u, u);
	_Pupd = MatrixXd::Zero(u, u);
	_G = MatrixXd::Zero(u, 6);
	_Qw = MatrixXd::Zero(6, 6);
	_scaleFactor = 1.0;
	
	// Transition matrix
	Transition(dT, IMU._Ne, IMU._Fe, IMU._Cbe);

	// Process Noise Covariance
	_Qw(0, 0) = pow(0.013, 2); _Qw(1, 1) = pow(0.013, 2); _Qw(2, 2) = pow(0.013, 2);
	_Qw(3, 3) = pow(0.160, 2); _Qw(4, 4) = pow(0.160, 2); _Qw(5, 5) = pow(0.160, 2);
	ProcessNoiseCoeff(dT, IMU._Cbe);

	// State variance
	_Pupd(0, 0) = pow(0.250, 2); _Pupd(1, 1) = pow(0.250, 2); _Pupd(2, 2) = pow(0.250, 2);
	_Pupd(3, 3) = pow(0.050, 2); _Pupd(4, 4) = pow(0.050, 2); _Pupd(5, 5) = pow(0.050, 2);
	_Pupd(6, 6) = pow(0.001, 2); _Pupd(7, 7) = pow(0.001, 2); _Pupd(8, 8) = pow(0.001, 2);
	_Pupd(9, 9) = pow(0.0007, 2); _Pupd(10, 10) = pow(0.0007, 2); _Pupd(11, 11) = pow(0.0007, 2);
	_Pupd(12, 12) = pow(0.004, 2); _Pupd(13, 13) = pow(0.003, 2); _Pupd(14, 14) = pow(0.003, 2);
}
LooseKF::~LooseKF() {}

void LooseKF::clearKF() {
	_Xpre.resize(0);
	_Xupd.resize(0);
	_F.resize(0, 0);
	_G.resize(0, 0);
	_Q.resize(0, 0);
	_Qw.resize(0, 0);
	_Ppre.resize(0, 0);
	_Pupd.resize(0, 0);
}