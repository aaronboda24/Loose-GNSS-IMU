/*
* IMUmechECEF.h
* ECEF Mechanization of IMU
*  Created on: Sept 10, 2018
*      Author: Aaron Boda
*/

#ifndef IMUMECHECEF_H_
#define IMUMECHECEF_H_

#include "pch.h"
#include <list>
#include "DCM.h"
#include "GeoUtils.h"
#include "MatrixUtils.h"

class IMUmechECEF
{
public:
	// CONSTRUCTOR
	IMUmechECEF();
	// DESTRUCTOR
	~IMUmechECEF();

	// Methods
	Eigen::VectorXd gravityECEF(double X, double Y, double Z);
	Eigen::MatrixXd TensorGravGrad(double X, double Y, double Z);
	double HeadingCorrection(double imuHeading, double gpsHeading);
	void MechanizerECEF(double dT, std::vector<double> fib, std::vector<double> wib, std::vector<double> LLH);
	void InitializeMechECEF(std::vector<double> iniPOS, std::vector<double> iniLLH, std::vector<double> iniVEL, std::vector<double> iniATT, std::vector<double> fbias, std::vector<double> gbias);

	// Attributes
	double _Heading_diff;
	std::vector<double> _fbias;
	std::vector<double> _gbias;
	std::vector<double> _pos;
	std::vector<double> _vel;
	std::vector<double> _att;
	std::vector<double> _LLHo;
	std::vector<double> _ECEFo;
	Eigen::MatrixXd _Cbe;
	Eigen::MatrixXd _Cbn;
	Eigen::MatrixXd _Obib;
	Eigen::MatrixXd _Oeie;
	Eigen::MatrixXd _Ne;
	Eigen::MatrixXd _Fe;
	Eigen::VectorXd _Lxyz;; // IMU to GPS lever arm in the body frame


private:
	// Functions

};

#endif /* IMUMECHECEF_H_ */


