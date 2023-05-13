//---------------------------------------------------------------//
// Name: IK_3_parallel_2_intersecting.h
// Author: Amar Maksumic
// Date: 03/15/2023
// Purpose: Port of the IK_3_parallel_2_intersecting files
//---------------------------------------------------------------//


#ifndef __IK_3_parallel_2_intersecting_h__
#define __IK_3_parallel_2_intersecting__

#include <eigen3/Eigen/Dense>
#include <vector>

struct Kin {
	Eigen::Matrix<double, 3, 6> H;
	Eigen::Matrix<double, 3, 7> P;
	Eigen::Matrix<double, 1, 6> joint_type;
};

struct Soln {
	std::vector<std::vector<double>> Q;
	std::vector<std::vector<bool>> is_LS_vec;
};

void IK_3_parallel_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kin& kin, Soln& soln);


#endif