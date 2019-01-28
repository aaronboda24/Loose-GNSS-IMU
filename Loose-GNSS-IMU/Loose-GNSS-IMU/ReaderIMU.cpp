/*
* ReaderIMU.cpp
* Read and organize IMU sensor input in epochwise manner
*  Created on: Sept 27, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "ReaderIMU.h"
using namespace std;

// CONSTRUCTOR AND DESTRUCTOR DEFINITIONS
ReaderIMU::ReaderIMU() {}
ReaderIMU::~ReaderIMU() {}

// To clear contents in observation data structure
void ReaderIMU::clearObs() {
	_IMUdata.Acc.clear(); _IMUdata.Gyr.clear();
	_IMUdata.Ax, _IMUdata.Ay, _IMUdata.Az = NULL;
	_IMUdata.Gx, _IMUdata.Gy, _IMUdata.Gz = NULL;
	_IMUdata.imuTime = NULL;
}

// This function extracts and stores epochwise observations from file
void ReaderIMU::obsEpoch(ifstream& infile) {
	// Initializing Variables
	double epochTime = 0;
	vector<double> acc;
	vector<double> gyr;
	// Read first line
	string line; getline(infile, line);
	if (!line.empty()) {
		// Split words in the line
		istringstream iss(line);
		vector<string> words{ istream_iterator<string>{iss}, istream_iterator<string>{} };
		// Extract observation time
		epochTime = stod(words[0]);
		// Organizing Acceleration Observations
		acc.push_back(stod(words[1])); acc.push_back(stod(words[2])); acc.push_back(stod(words[3]));
		// Organizing Gyroscope Observations
		gyr.push_back(stod(words[4])); gyr.push_back(stod(words[5])); gyr.push_back(stod(words[6]));
	}
	// Organize IMU Data structure
	_IMUdata.imuTime = epochTime;
	_IMUdata.Acc = acc;
	_IMUdata.Gyr = gyr;
	_IMUdata.Ax = acc[0]; _IMUdata.Ay = acc[1]; _IMUdata.Az = acc[2];
	_IMUdata.Gx = gyr[0]; _IMUdata.Gy = gyr[1]; _IMUdata.Gz = gyr[2];
}

