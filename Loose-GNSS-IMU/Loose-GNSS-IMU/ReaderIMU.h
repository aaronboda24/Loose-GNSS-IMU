#pragma once
/*
* ReaderIMU.h
* Read and organize IMU sensor input in epochwise manner
*  Created on: Sept 27, 2018
*      Author: Aaron Boda
*/

#include "pch.h"

#ifndef READERIMU_H_
#define READERIMU_H_

class ReaderIMU
{
public:
	// CONSTRUCTOR
	ReaderIMU();
	// DESTRUCTOR
	~ReaderIMU();

	// Data Structures
	// To store observations in an epoch
	struct IMUEpochInfo {
		double imuTime;
		std::vector<double> Acc;
		std::vector<double> Gyr;
		double Ax, Ay, Az;
		double Gx, Gy, Gz;
	};

	// Attributes
	IMUEpochInfo _IMUdata;

	// Functions
	void clearObs();
	void obsEpoch(std::ifstream& infile);

private:

};

#endif /* READERIMU_H_ */
