//---------------------------------------------------------------//
// Name: sp_6.h
// Author: Amar Maksumic
// Date: 11/28/2022
// Purpose: Port of the subproblem_setups/sp_6.m file
//---------------------------------------------------------------//

#ifndef __sp_4_h_
#define __sp_4_h_

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Polynomials>
#include <iostream>
#include "../rand_cpp.h"

void find_quartic_roots(Eigen::Matrix<double, 5, 1>& coeffs, 
                        Eigen::Matrix<std::complex<double>, 4, 1>& roots);

void solve_2_ellipse_numeric(Eigen::Vector2d& xm1, Eigen::Matrix<double, 2, 2>& xn1, 
                             Eigen::Vector2d& xm2, Eigen::Matrix<double, 2, 2>& xn2,
                             Eigen::Matrix<double, 4, 1>& xi_1, Eigen::Matrix<double, 4, 1>& xi_2);

void sp6_setup(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
               Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2, 
               double& theta1, double& theta2);

bool sp6_run(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
             Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2, 
             double& theta1, double& theta2);

#endif