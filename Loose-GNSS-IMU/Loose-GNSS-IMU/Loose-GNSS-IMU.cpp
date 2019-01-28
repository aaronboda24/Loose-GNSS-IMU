// Loose-GNSS-IMU.cpp : Traditional integration strategy for GNSS-IMU, loosely coupled.
// Author: Aaron Boda

#include "pch.h"
#include "Loosely.h"

using namespace std;

// The program starts and ends inside main.
int main()
{
	clock_t start; start = clock();

	// *** Input/Output file path
	string filePathGNSS = "Input/GNSS.txt";
	string filePathIMU = "Input/IMU.txt";
	string filePathOUT = "Output/LOOSE-GNSS-IMU.txt";

	// *** Loosely-coupled integration of GNSS-IMU
	Loosely(filePathGNSS, filePathIMU, filePathOUT);

	// *** Time elapsed
	double	duration = (double)((clock() - start) / CLOCKS_PER_SEC);
	cout << "TIME TAKEN : " << duration << endl;
	return 0;
}

