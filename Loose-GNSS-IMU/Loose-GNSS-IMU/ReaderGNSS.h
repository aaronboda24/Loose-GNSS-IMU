#pragma once
/*
* ReaderGNSS.h
* Read and organize GNSS solution in epochwise manner
*  Created on: Jan 27, 2019
*      Author: Aaron Boda
*/

#include "pch.h"

class ReaderGNSS
{
public:
	ReaderGNSS();
	~ReaderGNSS();

	struct GNSSEpochInfo {
		double gpsTime;
		Eigen::VectorXd Pxyz;
		Eigen::VectorXd Vxyz;
		Eigen::MatrixXd CovPxyz;
		Eigen::MatrixXd CovVxyz;
		Eigen::MatrixXd CovPVxyz;
	};

	// Attribute
	const int _HeaderLines = 8;

	GNSSEpochInfo _GNSSdata;

	// Functions
	void clearObs();
	void readEpoch(std::ifstream& infile);
	void readHeader(std::ifstream& infile);

};

