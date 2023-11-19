//---------------------------------------------------------------//
// Name: IK_spherical_2_parallel.h
// Author: Runbin Chen
// Date: 04/01/2023
//---------------------------------------------------------------//

#ifndef __IK_2_parallel_h_
#define __IK_2_parallel_h_

#include "../utils.h"
#include <eigen3/Eigen/Dense>

Solution<6> IK_2_parallel(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin);

#endif
