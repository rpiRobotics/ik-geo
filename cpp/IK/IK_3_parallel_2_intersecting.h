//---------------------------------------------------------------//
// Name: IK_3_parallel_2_intersecting.h
// Author: Amar Maksumic
// Date: 03/15/2023
// Purpose: Port of the IK_3_parallel_2_intersecting files
//---------------------------------------------------------------//


#ifndef __IK_3_parallel_2_intersecting_h__
#define __IK_3_parallel_2_intersecting_h__

#include "../utils.h"
#include <eigen3/Eigen/Dense>

Solution<6> IK_3_parallel_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin);

#endif
