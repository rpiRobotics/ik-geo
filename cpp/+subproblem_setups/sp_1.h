//---------------------------------------------------------------//
// Name: sp_1.h
// Author: Ashton Ropp, Runbin Chen, Amar Maksumic
// Date: 10/15/2022
// Purpose: Port of the subproblem_setups/sp_1.m file
//---------------------------------------------------------------//

#ifndef __sp_1_h_
#define __sp_1_h_

#include <eigen3/Eigen/Dense>

void sp1_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& k, double& theta);

void sp1_setup_LS(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                  Eigen::Vector3d& k, double& theta);

bool sp1_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
             const Eigen::Vector3d& k, double& theta);

double sp1_error(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
                 const Eigen::Vector3d& k, double& theta);

#endif