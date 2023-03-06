//---------------------------------------------------------------//
// Name: sp_5.h
// Author: Runbin Chen
// Date: 12/16/2022
//---------------------------------------------------------------//

#ifndef __sp_5_h_
#define __sp_5_h_

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <complex>
#include "../rand_cpp.h"

void sp_5_setup(Eigen::Vector3d &p0, Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, Eigen::Vector3d &k1, Eigen::Vector3d &k2, Eigen::Vector3d &k3);

void cone_polynomials(const Eigen::Vector3d &p0_i, const Eigen::Vector3d &k_i, const Eigen::Vector3d &p_i, const Eigen::Vector3d &p_i_s, const Eigen::Vector3d &k2, 
					  Eigen::Matrix<double, 1, 2>& P, Eigen::Matrix<double, 1, 3>& R);

Eigen::Matrix<double, 1, 3> convolution_2(Eigen::Matrix<double, 1, 2> &v1, Eigen::Matrix<double, 1, 2> &v2);

Eigen::Matrix<double, 1, 5> convolution_3(Eigen::Matrix<double, 1, 3> &v1, Eigen::Matrix<double, 1, 3> &v2);

std::vector<std::complex<double>> quartic_roots(const std::vector<double>& poly);

void sp_5(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, 
		  const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, const Eigen::Vector3d &k3, 
		  std::vector<double> &theta1, std::vector<double> &theta2, std::vector<double> &theta3);

#endif