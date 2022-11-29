//---------------------------------------------------------------//
// Name: sp_6.h
// Author: Amar Maksumic
// Date: 11/28/2022
// Purpose: Port of the subproblem_setups/sp_6.m file
//---------------------------------------------------------------//

#ifndef __sp_4_h_
#define __sp_4_h_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../rand_cpp.h"

void sp6_setup(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
               Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2, 
               double& theta1, double& theta2);

bool sp6_run(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
             Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2);

#endif