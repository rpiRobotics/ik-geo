//---------------------------------------------------------------//
// Name: IK_spherical_2_intersecting.h
// Author: Amar Maksumic
// Date: 02/03/2022
// Purpose: Port of the IK_spherical_2_intersecting files
//---------------------------------------------------------------//


#ifndef __IK_spherical_2_intersecting_h__
#define __IK_spherical_2_intersecting_h__

#include <eigen3/Eigen/Dense>
#include "../rand_cpp.h"
#include "../+subproblem_setups/sp_1.h"
#include "../+subproblem_setups/sp_2.h"
#include "../+subproblem_setups/sp_3.h"
#include "../+subproblem_setups/sp_4.h"

struct Kin {
	Eigen::Matrix<double, 3, 7> H;
	Eigen::Matrix<double, 3, 7> P;
};

struct Soln {
	std::vector<std::vector<double>> Q;
	std::vector<bool> is_LS_vec;
};

void setup(Eigen::Matrix<double, 3, 7>& H, Eigen::Matrix<double, 3, 7>& P, 
           Eigen::Matrix<double, 6, 1>& Q, Eigen::Matrix<double, 1, 6>& joint_type);

void setup_LS(Eigen::Matrix<double, 3, 7>& H, Eigen::Matrix<double, 3, 7>& P, 
           Eigen::Matrix<double, 6, 1>& Q, Eigen::Matrix<double, 1, 6>& joint_type);

void error();

void IK_spherical_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kin& kin, 
                                 Eigen::Matrix<double, 6, Eigen::Dynamic>& Q, Eigen::Matrix<double, 5, Eigen::Dynamic>& Q_LS);


#endif