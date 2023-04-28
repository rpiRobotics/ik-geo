//---------------------------------------------------------------//
// Name: sp_5.h
// Author: Runbin Chen
// Date: 12/16/2022
//---------------------------------------------------------------//

#ifndef __sp_5_h_
#define __sp_5_h_

#include <vector>
#include <eigen3/Eigen/Dense>

void sp_5_setup(Eigen::Vector3d &p0, Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, Eigen::Vector3d &k1, Eigen::Vector3d &k2, Eigen::Vector3d &k3);

void sp_5(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, 
		  const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, const Eigen::Vector3d &k3, 
		  std::vector<double> &theta1, std::vector<double> &theta2, std::vector<double> &theta3);

#endif