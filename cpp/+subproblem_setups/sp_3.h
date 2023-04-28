//---------------------------------------------------------------//
// Name: sp_3.h
// Author: Runbin Chen
// Date: 11/06/2022
//---------------------------------------------------------------//

#ifndef __sp_3_h_
#define __sp_3_h_

#include <vector>
#include <eigen3/Eigen/Dense>

void sp_3_setup(Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &k, double &d);

bool sp_3(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &k, const double &d, 
		  std::vector<double> &theta);

#endif