//---------------------------------------------------------------//
// Name: sp_6.h
// Author: Amar Maksumic
// Date: 11/28/2022
// Purpose: Port of the subproblem_setups/sp_6.m file
//---------------------------------------------------------------//

#ifndef __sp_6_h_
#define __sp_6_h_

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Polynomials>
#include <iostream>
#include "../rand_cpp.h"

void sp6_setup(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
               Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2, 
               std::vector<double>& theta1, std::vector<double>& theta2);

void sp6_run(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
             Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2, 
             std::vector<double> &theta1, std::vector<double> &theta2);

#endif
