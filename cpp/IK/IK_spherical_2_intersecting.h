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

struct Kin {
	Eigen::Matrix<double, 3, 6> H;
	Eigen::Matrix<double, 3, 7> P;
	Eigen::Matrix<double, 1, 6> joint_type;
};

struct Soln {
	std::vector<std::vector<double>> Q;
	std::vector<bool> is_LS_vec;
};

void fwdkin(const Kin& kin, const Soln& soln, 
            Eigen::Matrix<double, 3, 1>& p, 
						Eigen::Matrix<double, 3, 3>& R);

void setup(Kin& kin, Soln& soln,
					 Eigen::Matrix<double, 3, 1>& T, 
					 Eigen::Matrix<double, 3, 3>& R);

void setup_LS(Kin& kin, Soln& soln,
					 Eigen::Matrix<double, 3, 1>& T, 
					 Eigen::Matrix<double, 3, 3>& R);

void error();

void IK_spherical_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kin& kin, 
                                 Eigen::Matrix<double, 6, Eigen::Dynamic>& Q, Eigen::Matrix<double, 5, Eigen::Dynamic>& Q_LS);


#endif