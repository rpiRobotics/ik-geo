//---------------------------------------------------------------//
// Name: sp_5.h
// Author: Runbin Chen
// Date: 11/10/2022
//---------------------------------------------------------------//

#ifndef __sp_5_h_
#define __sp_5_h_

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <complex>
#include "../rand_cpp.h"

using namespace Eigen;

void sp_5_setup(Vector3d &p0, Vector3d &p1, Vector3d &p2, Vector3d &p3, Vector3d &k1, Vector3d &k2, Vector3d &k3) {
	p0 = rand_vec();
	p1 = rand_vec();
	p2 = rand_vec();
	p3 = rand_vec();
	k1 = rand_normal_vec();
	k2 = rand_normal_vec();
	k3 = rand_normal_vec();
}

void cone_polynomials(Vector3d &p0_i, Vector3d &k_i, Vector3d &p_i, Vector3d &p_i_s, Vector3d &k2
					  Matrix<double, 1, 2> &P, Matrix<double, 1, 3> &R) {
	Matrix<double, 3, 1> kiXk2 = k_i.cross(k2);
	Matrix<double, 3, 1> kiXkiXk2 = k_i.cross(kiXk2);
	double norm_kiXk2_sq = kiXk2.dot(kiXk2);

	Matrix<double, 3, 1> kiXpi = k_i.cross(p_i);
	double norm_kiXpi_sq = kiXpi.dot(kiXpi);

	double delta = k2.dot(p_i_s);
	double alpha = (p0_i.transpose() * kiXkiXk2 / norm_kiXk2_sq)(0, 0);
	double beta = (p0_i.transpose() * kiXk2 / norm_kiXk2_sq)(0, 0);

	double P_const = norm_kiXpi_sq + p_i_s.dot(p_i_s) + 2*alpha*delta;
	P << -2*alpha, P_const;

	R << -1, 2*delta, -pow(delta, 2);
	R(0, 2) = R(0, 2) + norm_kiXpi_sq*norm_kiXk2_sq;
	R = pow(2*beta, 2) * R;
	return;
}

Matrix<double, 1, 3> convolution_2(Matrix<double, 1, 2> &v1, Matrix<double, 1, 2> &v2) {
	Matrix<double, 1, 3> res;
	res << v1(0, 0)*v2(0, 0), v1(0, 0)*v2(0, 1)+v1(0, 1)*v2(0, 0), v1(0, 1)*v2(0, 1);
	return res;
}

Matrix<double, 1, 5> convolution_3(Matrix<double, 1, 3> &v1, Matrix<double, 1, 3> &v2) {
	Matrix<double, 1, 5> res;
	res << v1(0, 0)*v2(0, 0), v1(0, 1)*v2(0, 0)+v1(0, 0)*v2(0, 1), v1(0, 0)*v2(0, 2)+v1(0, 1)*v2(0, 1)+v1(0, 2)*v2(0, 0), 
		   v1(0, 1)*v2(0, 2)+v1(0, 2)*v2(0, 1), v1(0, 2)*v2(0, 2);
	return res;
}

std::vector<double> find_real_roots(Matrix<double, 1, 5> &eqn) {
	double a = eqn(0, 0), b = eqn(0, 1), c = eqn(0, 2), d = eqn(0, 3), e = eqn(0, 4);

	double B = b / a, C = c / a, D = d / a, E = e / a;

	double p = -B * B * 0.375 + C;  // double p = -3.0 / 8 * B * B + 1.0;
	double q = pow(B, 3) * 0.125 - B * C * 0.5 + D;
	double r = -pow(B, 4) * 3 / 256 + B * B * C * 0.0625 - B * D * 0.25 + E;

	double tmp1 = 1. / 6 * r * p - 1. / 216 * pow(p, 3) - 1. / 16 * q * q;
	double tmp2 = pow(1. / 3 * r + 1. / 36 * p * p, 3);
	std::complex<double> y1 = p/6 - pow(tmp1-sqrt(pow(tmp1,2)-tmp2+0i), 1./3) - pow(tmp1+sqrt(pow(tmp1,2)-tmp2+0i), 1./3);

	std::complex<double> tmp3 = sqrt(2.*y1 - p);
	std::complex<double> tmp4 = -2.0 * q / tmp3;

	std::complex<double> x1 = 0.5 * (tmp3 + sqrt(-p - 2.*y1 + tmp4)) - B * 0.25;
	std::complex<double> x2 = 0.5 * (tmp3 - sqrt(-p - 2.*y1 + tmp4)) - B * 0.25;
	std::complex<double> x3 = 0.5 * (-tmp3 + sqrt(-p - 2.*y1 - tmp4)) - B * 0.25;
	std::complex<double> x4 = 0.5 * (-tmp3 - sqrt(-p - 2.*y1 - tmp4)) - B * 0.25;

	std::vector<double> res;
	if (fabs(x1.imag()) < ZERO_THRESH) res.push_back(x1.real());
	if (fabs(x2.imag()) < ZERO_THRESH) res.push_back(x2.real());
	if (fabs(x3.imag()) < ZERO_THRESH) res.push_back(x3.real());
	if (fabs(x4.imag()) < ZERO_THRESH) res.push_back(x4.real());

	return res;
}

void sp_5(Vector3d &p0, Vector3d &p1, Vector3d &p2, Vector3d &p3, Vector3d &k1, Vector3d &k2, Vector3d &k3, 
		  std::vector<double> &theta1, std::vector<double> &theta2, std::vector<double> &theta3) {
	theta1 = std::vector<double>(0);
	theta2 = std::vector<double>(0);
	theta3 = std::vector<double>(0);
	int i_soln = 0;

	Matrix<double, 3, 1> p1_s = p0 + k1*k1.transpose()*p1;
	Matrix<double, 3, 1> p3_s = p2 + k3*k3.transpose()*p3;

	double delta1 = k2.dot(p1_s);
	double delta3 = k2.dot(p3_s);

	Matrix<double, 1, 2> P_1, P_3;
	Matrix<double, 1, 3> R_1, R_3;
	cone_polynomials(p0, k1, p1, p1_s, k2, P_1, R_1);
	cone_polynomials(p2, k3, p3, p3_s, k2, P_3, R_3);
	
	Matrix<double, 1, 2> P_13 = P_1 - P_3;
	Matrix<double, 1, 3> P_13_sq = convolution_2(P_13, P_13);

	Matrix<double, 1, 3> RHS = R_3 - R_1 - P_13_sq;

	Matrix<double, 1, 5> EQN = convolution_3(RHS, RHS) - 4*convolution_3(P_13_sq, R_1);

	std::vector<double> H_vec = find_real_roots(EQN);

	Matrix<double, 3, 1> KxP1 = k1.cross(p1);
	Matrix<double, 3, 1> KxP3 = k3.cross(p3);
	Matrix<double, 3, 2> A_1;
	A_1 << KxP1, -k1.cross(KxP1);
	Matrix<double, 3, 2> A_3;
	A_3 << KxP3, -k3.cross(KxP3);

	int signs[2][4];
	signs[0] = {1, 1, -1, -1};
	signs[1] = {1, -1, 1, -1};
	Matrix<double, 2, 2> J;
	J << 0, 1, 
		-1, 0;

	for (int i_H = 0; i_H < (int)H_vec.size(); i_H ++ ) {
		double H = H_vec[i_H];

		Matrix<double, 2, 1> const_1 = A_1.transpose() * k2 * (H - delta1);
		Matrix<double, 2, 1> const_3 = A_3.transpose() * k2 * (H - delta3);
		Matrix<double, 2, 1> pm_1 = J*A_1.transpose()*k2*sqrt(pow((A_1.transpose()*k2).norm(), 2) - pow(H-delta1, 2));
		Matrix<double, 2, 1> pm_3 = J*A_3.transpose()*k2*sqrt(pow((A_3.transpose()*k2).norm(), 2) - pow(H-delta3, 2));
		
		for (int i_sign = 0; i < 4; i ++ ) {
			int sign_1 = signs[0][i_sign];
			int sign_3 = signs[1][i_sign];

			Matrix<double, 2, 1> sc1 = const_1 + (double)sign_1 * pm_1;
			sc1 = sc1 / pow((A_1.transpose()*k2).norm(), 2);

			Matrix<double, 2, 1> sc3 = const_3 + (double)sign_3 * pm_3;
			sc3 = sc3 / pow((A_3.transpose()*k2).norm(), 2);

			Matrix<double, 3, 1> v1 = A_1*sc1 + p1_s;
			Matrix<double, 3, 1> v3 = A_3*sc3 + p3_s;
			
			if (fabs((v1-H*k2).norm() - (v3-H*k2).norm()) < 1e-6) {
        		i_soln ++ ;
				theta1.push_back(atan2(sc1(0, 0), sc1(1, 0)));
				// theta2[i_soln] = subproblem.sp_1(v3, v1, k2);
        		theta3.push_back(atan2(sc3(0, 0), sc3(1, 0)));
			}
		}

	}
}

#endif