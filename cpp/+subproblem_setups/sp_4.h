//---------------------------------------------------------------//
// Name: sp_4.h
// Author: Amar Maksumic
// Date: 11/15/2022
// Purpose: Port of the subproblem_setups/sp_4.m file
//---------------------------------------------------------------//

#ifndef __sp_4_h_
#define __sp_4_h_

#include <vector>
#include <eigen3/Eigen/Dense>

bool sp4_run(const Eigen::Vector3d& p, 
             const Eigen::Vector3d& k, 
             const Eigen::Vector3d& h, 
             const double& d, 
             std::vector<double>& theta);

#endif