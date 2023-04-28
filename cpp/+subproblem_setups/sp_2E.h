//---------------------------------------------------------------//
// Name: sp_2E.h
// Author: Runbin Chen
// Date: 11/05/2022
//---------------------------------------------------------------//

#ifndef __sp_2E_h_
#define __sp_2E_h_

#include <eigen3/Eigen/Dense>

void sp_2E_setup(Eigen::Vector3d &p0, Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &k1, Eigen::Vector3d &k2, 
		  double &theta1, double &theta2);

void sp_2E(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, 
		  double &theta1, double &theta2);

#endif