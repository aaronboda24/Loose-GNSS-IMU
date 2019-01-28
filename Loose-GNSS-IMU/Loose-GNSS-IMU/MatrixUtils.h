#pragma once
/*
* MatrixUtils.h
* Some custom functions used for eigen matrix/vector manipulation
*  Created on: Jun 10, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "MathUtils.h"

#ifndef MATRIXUTILS_H_
#define MATRIXUTILS_H_

// Functions
Eigen::MatrixXd Rx(double ang);
Eigen::MatrixXd Ry(double ang);
Eigen::MatrixXd Rz(double ang);
void NormaliseAttitude(Eigen::VectorXd &Vec);
void NormaliseAttitudeOnly(Eigen::VectorXd &Vec);
Eigen::MatrixXd SkewMat(std::vector<double> Vec);
Eigen::MatrixXd SkewMat(Eigen::VectorXd Vec);
Eigen::VectorXd std2eigVector(std::vector<double> A);
std::vector<double> eigVector2std(Eigen::VectorXd A);
Eigen::VectorXd CrossProd(Eigen::VectorXd A, Eigen::VectorXd B);
std::vector<double> stdVecAdd(std::vector<double> V1, std::vector<double> V2);
std::vector<double> stdVecSub(std::vector<double> V1, std::vector<double> V2);
std::vector<double> Transformer(std::vector<double> v, Eigen::MatrixXd RotMat);

#endif /* MATRIXUTILS_H_ */
