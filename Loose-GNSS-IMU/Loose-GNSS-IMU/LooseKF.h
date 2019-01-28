/*
* LooseKF.h
* 21 state Kalman Filter for GNSS-IMU Loosely Coupled Integration
*  Created on: Sept 12, 2018
*      Author: Aaron Boda
*/

#ifndef LOOSEKF_H_
#define LOOSEKF_H_

#include "pch.h"
#include "MatrixUtils.h"
#include "IMUmechECEF.h"
#include "ReaderGNSS.h"

class LooseKF
{
public:
	// CONSTRUCTOR
	LooseKF(IMUmechECEF &IMU, double dT);
	// DESTRUCTOR
	~LooseKF();

	struct KFupd {
		std::vector<double> posXYZ;
		std::vector<double> velXYZ;
		std::vector<double> attXYZ;
		std::vector<double> df;
		std::vector<double> dw;
	};

	// Attributes
	KFupd sol;
	Eigen::VectorXd _Xpre; // Predicted State Vector
	Eigen::VectorXd _Xupd; // Measurement Updated State Vector 
	Eigen::MatrixXd _F; // State Transition Matrix
	Eigen::MatrixXd _G; // Process Noise Coefficient Matrix
	Eigen::MatrixXd _Qw; // Process Noise Covariance
	Eigen::MatrixXd _Q; // Process Noise Covariance
	Eigen::MatrixXd _Ppre; // State Covariance (Predicted)
	Eigen::MatrixXd _Pupd; // State Covariance (Updated)
	Eigen::VectorXd _Zobs; // IMU Observation Vector
	Eigen::MatrixXd _Robs; // IMU Observation Variance Matrix
	double _scaleFactor; // Scale Factor of covariance matrix
	
	// Functions
	void Transition(double dT, Eigen::MatrixXd Ne, Eigen::MatrixXd Fe, Eigen::MatrixXd Cbe);
	void ProcessNoiseCoeff(double dT, Eigen::MatrixXd Cbe);
	void SetObs(ReaderGNSS &GNSS, IMUmechECEF &IMU);
	void Filter(IMUmechECEF &IMU);
	void clearKF();

private:
	// Functions

};

#endif /* LOOSEKF_H_ */