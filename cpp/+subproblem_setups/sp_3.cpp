//---------------------------------------------------------------//
// Name: sp_3.cpp
// Author: Runbin Chen
// Date: 11/06/2022
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include "sp_3.h"

using namespace Eigen;

void sp_3_setup(Vector3d &p1, Vector3d &p2, Vector3d &k, double &d) {
	p1 = rand_vec();
	p2 = rand_vec();
	k = rand_normal_vec();
	d = rand_0to1();
}

// return is_LS
bool sp_3(Vector3d &p1, Vector3d &p2, Vector3d &k, double &d, 
		  std::vector<double> &theta) {
	Matrix<double, 3, 1> KxP = k.cross(p1);

	Matrix<double, 3, 2> A_1;
	A_1 << KxP, -k.cross(KxP);
	Matrix<double, 1, 2> A;
	A = -2 * p2.transpose() * A_1;
	double norm_A_sq = A.dot(A);
	double norm_A = sqrt(norm_A_sq);

	double b = pow(d, 2) - pow((p2-k*k.transpose()*p1).norm(), 2) - pow(KxP.norm(), 2);

	Matrix<double, 2, 1> x_ls = A_1.transpose() * (-2*p2*b/norm_A_sq);

	theta.clear();
	if (x_ls.dot(x_ls) > 1) {
		theta.push_back(atan2(x_ls(0, 0), x_ls(1, 0)));
		return true;
	}

	double xi = sqrt(1-pow(b, 2)/norm_A_sq);

	Matrix<double, 2, 1> A_perp_tilde, A_perp;
	A_perp_tilde << A(1, 0), -A(0, 0);
	A_perp = A_perp_tilde / norm_A;

	Matrix<double, 2, 1> sc_1, sc_2;
	sc_1 = x_ls + xi*A_perp;
	sc_2 = x_ls - xi*A_perp;

	theta.push_back(atan2(sc_1(0, 0), sc_1(1, 0)));
	theta.push_back(atan2(sc_2(0, 0), sc_2(1, 0)));

	return false;
}