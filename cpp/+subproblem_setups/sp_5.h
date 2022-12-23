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

void sp_5_setup(Vector3d &p0, Vector3d &p1, Vector3d &p2, Vector3d &p3, Vector3d &k1, Vector3d &k2, Vector3d &k3);

void cone_polynomials(Vector3d &p0_i, Vector3d &k_i, Vector3d &p_i, Vector3d &p_i_s, Vector3d &k2, 
					  Matrix<double, 1, 2> &P, Matrix<double, 1, 3> &R);

Matrix<double, 1, 3> convolution_2(Matrix<double, 1, 2> &v1, Matrix<double, 1, 2> &v2);

Matrix<double, 1, 5> convolution_3(Matrix<double, 1, 3> &v1, Matrix<double, 1, 3> &v2);

std::vector<double> find_real_roots(Matrix<double, 1, 5> &eqn);

void sp_5(Vector3d &p0, Vector3d &p1, Vector3d &p2, Vector3d &p3, Vector3d &k1, Vector3d &k2, Vector3d &k3, 
		  std::vector<double> &theta1, std::vector<double> &theta2, std::vector<double> &theta3);

#endif