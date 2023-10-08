#ifndef IK_SPHERICAL_H_
#define IK_SPHERICAL_H_

#include <eigen3/Eigen/Dense>
#include "../utils.h"

Solution IK_spherical(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kinematics& kin);

#endif // IK_SPHERICAL_H_
