//---------------------------------------------------------------//
// Name: sp_6.h
// Author: Amar Maksumic
// Date: 11/28/2022
// Purpose: Port of the subproblem_setups/sp_6.m file
//---------------------------------------------------------------//

#ifndef __sp_6_h_
#define __sp_6_h_

#include <vector>
#include <eigen3/Eigen/Dense>

void sp6_run(Eigen::Matrix<double, 3, 4>& p, 
             Eigen::Matrix<double, 3, 4>& k, 
             Eigen::Matrix<double, 3, 4>& h, 
             double& d1, double& d2, 
             std::vector<double> &theta1, std::vector<double> &theta2);

#endif
