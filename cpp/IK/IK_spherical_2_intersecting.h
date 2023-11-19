//---------------------------------------------------------------//
// Name: IK_spherical_2_intersecting.h
// Author: Amar Maksumic
// Date: 02/03/2022
// Purpose: Port of the IK_spherical_2_intersecting files
//---------------------------------------------------------------//


#ifndef __IK_spherical_2_intersecting_h__
#define __IK_spherical_2_intersecting_h__

#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utils.h"

struct Soln {
	std::vector<std::vector<double> > Q;
	std::vector<bool> is_LS_vec;
};

void fwdkin(const Kinematics<6, 7>& kin, const Soln& soln,
            Eigen::Matrix<double, 3, 1>& p, 
						Eigen::Matrix<double, 3, 3>& R);

void setup(Kinematics<6, 7>& kin, Soln& soln,
					 Eigen::Matrix<double, 3, 1>& T, 
					 Eigen::Matrix<double, 3, 3>& R);

void setup_LS(Kinematics<6, 7>& kin, Soln& soln,
					 Eigen::Matrix<double, 3, 1>& T, 
					 Eigen::Matrix<double, 3, 3>& R);

void error();

Solution<6> IK_spherical_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin);

#endif
