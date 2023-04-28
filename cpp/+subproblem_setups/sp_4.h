//---------------------------------------------------------------//
// Name: sp_4.h
// Author: Amar Maksumic
// Date: 11/15/2022
// Purpose: Port of the subproblem_setups/sp_4.m file
//---------------------------------------------------------------//

#ifndef __sp_4_h_
#define __sp_4_h_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../rand_cpp.h"

void sp4_setup(Eigen::Vector3d& p, Eigen::Vector3d& k, 
               Eigen::Vector3d& h, double& d, std::vector<double>& theta);

void sp4_setup_LS(Eigen::Vector3d& p, Eigen::Vector3d& k, 
                  Eigen::Vector3d& h, double& d, Eigen::Vector2d& theta);

// return is_LS
bool sp4_run(const Eigen::Vector3d& p, const Eigen::Vector3d& k, 
             const Eigen::Vector3d& h, const double& d, std::vector<double>& theta);

double sp4_error(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                 Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
                 double& theta1, double& theta2);

#endif