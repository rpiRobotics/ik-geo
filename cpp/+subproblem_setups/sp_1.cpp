//---------------------------------------------------------------//
// Name: sp_1.h
// Author: Ashton Ropp, Runbin Chen
// Date: 10/15/2022
// Purpose: Port of the subproblem/sp_1.m file
//---------------------------------------------------------------//

#include "sp_1.h"

using namespace Eigen;

const double ZERO_THRESH = 1e-8;

// return is_LS
bool sp_1(Vector3d p1, Vector3d p2, Vector3d k, 
		  double &theta) {
	// p2 = rot(k, theta) * p1

	Matrix<double, 3, 1> KxP = k.cross(p1);
	Matrix<double, 3, 2> A;
	A << KxP, -k.cross(KxP);

	Vector2d x = A.transpose() * p2;

	theta = atan2(x(0), x(1));

	return fabs(p1.norm() - p2.norm()) > ZERO_THRESH || fabs(k.dot(p1) - k.dot(p2)) > ZERO_THRESH;
}

int main() {
	return 0;
}