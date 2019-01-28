/*
* AprioriGMIS.h
* Helps with initializing IMU states
*  Created on: Aug 26, 2018
*      Author: Aaron Boda
*/

#ifndef INITIALIZEIMU_H_
#define INITIALIZEIMU_H_

#include "pch.h"
#include "DCM.h"
#include "ReaderIMU.h"
#include "MatrixUtils.h"

class InitializeIMU
{
public:
	// CONSTRUCTOR
	InitializeIMU(std::ifstream& fin_imu, double iniTimeEnd, std::vector<double> LLH);
	// DESTRUCTOR
	~InitializeIMU();

	// Attributes
	double _roll;
	double _pitch;
	double _yaw;
	std::vector<double> _RPY;
	std::vector<double> _GYRavg;
	std::vector<double> _ACCavg;
	std::vector<double> _GYRbias;
	std::vector<double> _ACCbias;

private:
	// Functions

};

#endif /* INITIALIZEIMU_H_ */


