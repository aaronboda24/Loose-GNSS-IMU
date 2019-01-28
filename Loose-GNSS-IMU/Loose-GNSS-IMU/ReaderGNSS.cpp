/*
* ReaderGNSS.cpp
* Read and organize GNSS solution in epochwise manner
*  Created on: Jan 27, 2019
*      Author: Aaron Boda
*/

#include "pch.h"
#include "ReaderGNSS.h"
using namespace std;
using namespace Eigen;

// CONSTRUCTOR AND DESTRUCTOR DEFINITIONS
ReaderGNSS::ReaderGNSS() {}
ReaderGNSS::~ReaderGNSS() {}

// To clear contents in observation data structure
void ReaderGNSS::clearObs() {
	_GNSSdata.Pxyz.setZero(); _GNSSdata.Vxyz.setZero();
	_GNSSdata.CovPxyz.setZero(); _GNSSdata.CovVxyz.setZero();
	_GNSSdata.gpsTime = NULL;
}

// This function extracts and stores header information
void ReaderGNSS::readHeader(ifstream& infile) {
	// Do nothing with header for now...
	for (int i = 0; i <= _HeaderLines; i++) {
		string line; getline(infile, line);
	}
}

// This function extracts and stores epochwise observations from file
void ReaderGNSS::readEpoch(ifstream& infile) {
	// Initializing Variables
	double epochTime = 0;

	_GNSSdata.Pxyz = VectorXd::Zero(3);
	_GNSSdata.Vxyz = VectorXd::Zero(3);
	_GNSSdata.CovPxyz = MatrixXd::Zero(3, 3);
	_GNSSdata.CovVxyz = MatrixXd::Zero(3, 3);
	_GNSSdata.CovPVxyz = MatrixXd::Zero(3, 3);

	// Read header
	string line; getline(infile, line);

	if (!line.empty()) {
		// Split words in the line
		istringstream iss(line);
		vector<string> words{ istream_iterator<string>{iss}, istream_iterator<string>{} };
		
		// *** Organize GNSS Data structure
		// Extract observation time
		_GNSSdata.gpsTime = stod(words[0]);
		// Organizing GNSS Solution
		_GNSSdata.Pxyz(0) = stod(words[1]); _GNSSdata.Pxyz(1) = stod(words[2]); _GNSSdata.Pxyz(2) = stod(words[3]);
		_GNSSdata.Vxyz(0) = stod(words[10]); _GNSSdata.Vxyz(1) = stod(words[11]); _GNSSdata.Vxyz(2) = stod(words[12]);
		// Organizing Var-Cov
		_GNSSdata.CovPxyz(0, 0) = stod(words[4]); _GNSSdata.CovPxyz(0, 1) = stod(words[5]); _GNSSdata.CovPxyz(0, 2) = stod(words[6]);
		_GNSSdata.CovPxyz(1, 0) = stod(words[5]); _GNSSdata.CovPxyz(1, 1) = stod(words[7]); _GNSSdata.CovPxyz(1, 2) = stod(words[8]);
		_GNSSdata.CovPxyz(2, 0) = stod(words[6]); _GNSSdata.CovPxyz(2, 1) = stod(words[8]); _GNSSdata.CovPxyz(2, 2) = stod(words[9]);

		_GNSSdata.CovVxyz(0, 0) = stod(words[13]); _GNSSdata.CovVxyz(0, 1) = stod(words[14]); _GNSSdata.CovVxyz(0, 2) = stod(words[15]);
		_GNSSdata.CovVxyz(1, 0) = stod(words[14]); _GNSSdata.CovVxyz(1, 1) = stod(words[16]); _GNSSdata.CovVxyz(1, 2) = stod(words[15]);
		_GNSSdata.CovVxyz(2, 0) = stod(words[15]); _GNSSdata.CovVxyz(2, 1) = stod(words[17]); _GNSSdata.CovVxyz(2, 2) = stod(words[18]);

		_GNSSdata.CovPVxyz(0, 0) = stod(words[19]); _GNSSdata.CovPVxyz(1, 0) = stod(words[20]); _GNSSdata.CovPVxyz(2, 0) = stod(words[21]);
		_GNSSdata.CovPVxyz(1, 0) = stod(words[22]); _GNSSdata.CovPVxyz(1, 1) = stod(words[23]); _GNSSdata.CovPVxyz(1, 2) = stod(words[24]);
		_GNSSdata.CovPVxyz(2, 0) = stod(words[25]); _GNSSdata.CovPVxyz(2, 1) = stod(words[26]); _GNSSdata.CovPVxyz(2, 2) = stod(words[27]);

	}

}

