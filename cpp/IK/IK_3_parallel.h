#ifndef IK_3_PARALLEL_H_
#define IK_3_PARALLEL_H_

#include <eigen3/Eigen/Dense>
#include "../utils.h"

Solution<6> IK_3_parallel(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin);

#endif // IK_3_PARALLEL_H_
