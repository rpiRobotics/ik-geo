//---------------------------------------------------------------//
// Name: IK_spherical_2_parallel.h
// Author: Runbin Chen
// Date: 02/01/2023
//---------------------------------------------------------------//

#ifndef __IK_spherical_2_parallel_h_
#define __IK_spherical_2_parallel_h_

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

extern Eigen::Matrix<double, 3, Eigen::Dynamic> rot(Eigen::Matrix<double, 3, Eigen::Dynamic> k, double theta);

struct Kin {
	Eigen::Matrix<double, 3, 7> H;
	Eigen::Matrix<double, 3, 7> P;
};

struct Soln {
	std::vector<std::vector<double>> Q;
	std::vector<bool> is_LS_vec;
};

Soln IK_spherical_2_parallel(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kin& kin);

#endif