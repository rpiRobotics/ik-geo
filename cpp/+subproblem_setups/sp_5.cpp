//---------------------------------------------------------------//
// Name: sp_5.cpp
// Author: Runbin Chen
// Date: 12/16/2022
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <chrono>
#include "sp_1.h"
#include "sp_5.h"
#include "../read_csv.h"
#include "../helper.h"

using namespace std::complex_literals;

// const double ZERO_THRESH = 1e-8;

void sp_5_setup(Eigen::Vector3d &p0, Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, Eigen::Vector3d &k1, Eigen::Vector3d &k2, Eigen::Vector3d &k3) {
	p0 = rand_vec();
	p1 = rand_vec();
	p2 = rand_vec();
	p3 = rand_vec();
	k1 = rand_normal_vec();
	k2 = rand_normal_vec();
	k3 = rand_normal_vec();
}

// return is_LS
// bool sp1_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
//              const Eigen::Vector3d& k, double& theta){

// 	Eigen::Matrix<double, 3, 1> KxP = k.cross(p1);
// 	Eigen::Matrix<double, 3, 2> A;
// 	A << KxP, -k.cross(KxP);

// 	Eigen::Vector2d x = A.transpose() * p2;

// 	theta = atan2(x(0), x(1));

// 	return fabs(p1.norm() - p2.norm()) > ZERO_THRESH || fabs(k.dot(p1) - k.dot(p2)) > ZERO_THRESH;
// }

void sp_5(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, 
		  const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, const Eigen::Vector3d &k3, 
		  std::vector<double> &theta1, std::vector<double> &theta2, std::vector<double> &theta3) {
	theta1 = std::vector<double>(0);
	theta2 = std::vector<double>(0);
	theta3 = std::vector<double>(0);
	int i_soln = 0;

	Eigen::Matrix<double, 3, 1> p1_s = p0 + k1*k1.transpose()*p1;
	Eigen::Matrix<double, 3, 1> p3_s = p2 + k3*k3.transpose()*p3;

	double delta1 = k2.dot(p1_s);
	double delta3 = k2.dot(p3_s);

	Eigen::Matrix<double, 1, 2> P_1, P_3;
	Eigen::Matrix<double, 1, 3> R_1, R_3;
	cone_polynomials(p0, k1, p1, p1_s, k2, P_1, R_1);
	cone_polynomials(p2, k3, p3, p3_s, k2, P_3, R_3);
	
	Eigen::Matrix<double, 1, 2> P_13 = P_1 - P_3;
	Eigen::Matrix<double, 1, 3> P_13_sq = convolution_2(P_13, P_13);

	Eigen::Matrix<double, 1, 3> RHS = R_3 - R_1 - P_13_sq;

	Eigen::Matrix<double, 1, 5> EQN = convolution_3(RHS, RHS) - 4*convolution_3(P_13_sq, R_1);

	std::vector<std::complex<double>> all_roots = quartic_roots(EQN);
	std::vector<double> H_vec;
	for (int i = 0; i < (int)all_roots.size(); i ++ ) {
		if (fabs(all_roots[i].imag()) < 1e-6)
			H_vec.push_back(all_roots[i].real());
	}

	Eigen::Matrix<double, 3, 1> KxP1 = k1.cross(p1);
	Eigen::Matrix<double, 3, 1> KxP3 = k3.cross(p3);
	Eigen::Matrix<double, 3, 2> A_1;
	A_1 << KxP1, -k1.cross(KxP1);
	Eigen::Matrix<double, 3, 2> A_3;
	A_3 << KxP3, -k3.cross(KxP3);

	std::vector<std::vector<int>> signs(2);
	signs[0] = {1, 1, -1, -1};
	signs[1] = {1, -1, 1, -1};
	Eigen::Matrix<double, 2, 2> J;
	J << 0, 1, 
		-1, 0;

	for (int i_H = 0; i_H < (int)H_vec.size(); i_H ++ ) {
		double H = H_vec[i_H];

		Eigen::Matrix<double, 2, 1> const_1 = A_1.transpose() * k2 * (H - delta1);
		Eigen::Matrix<double, 2, 1> const_3 = A_3.transpose() * k2 * (H - delta3);
		Eigen::Matrix<double, 2, 1> pm_1 = J*A_1.transpose()*k2*sqrt(pow((A_1.transpose()*k2).norm(), 2) - pow(H-delta1, 2));
		Eigen::Matrix<double, 2, 1> pm_3 = J*A_3.transpose()*k2*sqrt(pow((A_3.transpose()*k2).norm(), 2) - pow(H-delta3, 2));
		
		for (int i_sign = 0; i_sign < 4; i_sign ++ ) {
			int sign_1 = signs[0][i_sign];
			int sign_3 = signs[1][i_sign];

			Eigen::Matrix<double, 2, 1> sc1 = const_1 + (double)sign_1 * pm_1;
			sc1 = sc1 / pow((A_1.transpose()*k2).norm(), 2);

			Eigen::Matrix<double, 2, 1> sc3 = const_3 + (double)sign_3 * pm_3;
			sc3 = sc3 / pow((A_3.transpose()*k2).norm(), 2);

			Eigen::Matrix<double, 3, 1> v1 = A_1*sc1 + p1_s;
			Eigen::Matrix<double, 3, 1> v3 = A_3*sc3 + p3_s;
			
			if (fabs((v1-H*k2).norm() - (v3-H*k2).norm()) < 1e-6) {
        		i_soln ++ ;
				theta1.push_back(atan2(sc1(0, 0), sc1(1, 0)));
				// theta2[i_soln] = subproblem.sp_1(v3, v1, k2);
				double theta;
				sp1_run(v3, v1, k2, theta);
				theta2.push_back(theta);
        		theta3.push_back(atan2(sc3(0, 0), sc3(1, 0)));
			}
		}

	}
}
