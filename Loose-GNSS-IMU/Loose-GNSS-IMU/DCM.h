#pragma once
/*
* DCM.h
* Direction Cosine Matrices
*  Created on: Aug 07, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "MathUtils.h"
#include "MatrixUtils.h"

#ifndef DCM_H_
#define DCM_H_

// Function Declarations
Eigen::MatrixXd llf2ecefDCM(double lat, double lon);
Eigen::MatrixXd ecef2llfDCM(double lat, double lon);
Eigen::MatrixXd b2llfDCM(double roll, double pitch, double yaw);
Eigen::MatrixXd llf2bDCM(double roll, double pitch, double yaw);
Eigen::MatrixXd b2eDCM(double lat, double lon, double r, double p, double y);
Eigen::MatrixXd e2bDCM(double lat, double lon, double r, double p, double y);
std::vector<double> dcm2euler(Eigen::MatrixXd Cnb);

#endif /* DCM_H_ */
