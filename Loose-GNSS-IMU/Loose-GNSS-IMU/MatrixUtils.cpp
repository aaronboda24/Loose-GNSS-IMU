/*
* MatrixUtils.cpp
* Compilation of custom functions used for eigen matrix manipulation
*  Created on: Jun 10, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "MatrixUtils.h"
using namespace std;
using namespace Eigen;

const double PI = 3.1415926535898;

// A function to build skew symmetric matrix
MatrixXd SkewMat(vector<double> Vec) {
	MatrixXd Skew = MatrixXd::Zero(3, 3);
	Skew(0, 1) = -Vec[2]; Skew(0, 2) = Vec[1];
	Skew(1, 0) = Vec[2]; Skew(1, 2) = -Vec[0];
	Skew(2, 0) = -Vec[1]; Skew(2, 1) = Vec[0];
	return Skew;
}

// A function to build skew symmetric matrix
MatrixXd SkewMat(VectorXd Vec) {
	MatrixXd Skew = MatrixXd::Zero(3, 3);
	Skew(0, 1) = -Vec(2); Skew(0, 2) = Vec(1);
	Skew(1, 0) = Vec(2); Skew(1, 2) = -Vec(0);
	Skew(2, 0) = -Vec(1); Skew(2, 1) = Vec(0);
	return Skew;
}

// A function to compute Cross Product
VectorXd CrossProd(VectorXd A, VectorXd B) {
	VectorXd C = VectorXd::Zero(3);
	C(0) = A[1] * B[2] - A[2] * B[1];
	C(1) = A[2] * B[0] - A[0] * B[2];
	C(2) = A[0] * B[1] - A[1] * B[0];
	return C;
}

// A function to convert eig vector to std vector
vector<double> eigVector2std(VectorXd A) {
	size_t n = A.size();
	vector<double> V;
	for (size_t i = 0; i < n; i++) {
		V.push_back(A(i));
	}
	return V;
}

// A function to convert std vector to eigen vector
VectorXd std2eigVector(vector<double> A) {
	size_t n = A.size();
	VectorXd V = VectorXd::Zero(n);
	for (size_t i = 0; i < n; i++) {
		V(i) = A[i];
	}
	return V;
}

// A function to adjust attitude values in state vector to be between 0 to 2PI
void NormaliseAttitude(VectorXd& States) {
	States(9)  = normalise(States(9) , -PI / 2., PI / 2.);
	States(10) = normalise(States(10), -PI / 2., PI / 2.);
	States(11) = normalise(States(11), -PI, PI);
}

// A function to adjust attitude values in state vector to be between 0 to 2PI
void NormaliseAttitudeOnly(VectorXd& States) {
	States(0) = normalise(States(0), -PI / 2., PI / 2.);
	States(1) = normalise(States(1), -PI / 2., PI / 2.);
	States(2) = normalise(States(2), -PI, PI);
}

// Standard Vector Addition
vector<double> stdVecAdd(vector<double> V1, vector<double> V2) {
	size_t n1 = V1.size();
	size_t n2 = V2.size();
	vector<double> Vadd;
	if (n1 == n2) {
		for(size_t i = 0; i < n1; i++){
			Vadd.push_back(V1.at(i) + V2.at(i));
		}
	}
	else {
		cout << "VECTOR SIZES NOT SAME, CANNOT PERFORM ADDITION, SEE USAGE OF FUNCTION stdVecAdd !!!" << endl;
	}
	return Vadd;
}

// Standard Vector Subtraction
vector<double> stdVecSub(vector<double> V1, vector<double> V2) {
	size_t n1 = V1.size();
	size_t n2 = V2.size();
	vector<double> Vsub;
	if (n1 == n2) {
		for (size_t i = 0; i < n1; i++) {
			Vsub.push_back(V1.at(i) - V2.at(i));
		}
	}
	else {
		cout << "VECTOR SIZES NOT SAME, CANNOT PERFORM SUBTRACTION, SEE USAGE OF FUNCTION stdVecSub !!!" << endl;
	}
	return Vsub;
}

/// Rotation Matrix Rx
MatrixXd Rx(double theta) {
	MatrixXd R(3, 3); R.setZero();
	R << 1, 0, 0,
		0, cos(theta), sin(theta),
		0, -sin(theta), cos(theta);
	return R;
}
// Rotation Matrix Ry
MatrixXd Ry(double theta) {
	MatrixXd R(3, 3); R.setZero();
	R << cos(theta), 0, -sin(theta),
		0, 1, 0,
		sin(theta), 0, cos(theta);
	return R;
}
// Rotation Matrix Rz
MatrixXd Rz(double theta) {
	MatrixXd R(3, 3); R.setZero();
	R << cos(theta), sin(theta), 0,
		-sin(theta), cos(theta), 0,
		0, 0, 1;
	return R;
}

// Transforms input vector using provided DCM
vector<double> Transformer(vector<double> v, MatrixXd DCM) {
	size_t n = v.size();
	VectorXd V = VectorXd::Zero(n);
	for (size_t i = 0; i < n; i++) {
		V(i) = v[i];
	}
	VectorXd W = VectorXd::Zero(n);
	W = DCM * V;
	vector<double> w;
	for (size_t i = 0; i < n; i++) {
		w.push_back(W(i));
	}
	return w;
}