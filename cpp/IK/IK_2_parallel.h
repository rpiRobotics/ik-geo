//---------------------------------------------------------------//
// Name: IK_spherical_2_parallel.h
// Author: Runbin Chen
// Date: 04/01/2023
//---------------------------------------------------------------//

#ifndef __IK_2_parallel_h_
#define __IK_2_parallel_h_

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

struct Kin {
	Eigen::Matrix<double, 3, 6> H;
	Eigen::Matrix<double, 3, 7> P;
	std::vector<int> joint_type;
};

struct Soln {
	std::vector<std::vector<double>> Q;
	std::vector<bool> is_LS_vec;
};

struct P {
	Kin kin;
	Eigen::Matrix<double, 3, 3> R;
	Eigen::Vector3d T;
};

void fwdkin(Kin& kin, std::vector<double>& theta, Eigen::Matrix<double, 3, 3>& R, Eigen::Vector3d& p);

void IK_2_parallel_setup(P &p, Soln& soln);

Soln IK_2_parallel(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kin& kin);

#endif