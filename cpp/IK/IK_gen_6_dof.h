#ifndef IK_GEN_6_DOF_H_
#define IK_GEN_6_DOF_H_

#include "../utils.h"
#include "IK_spherical.h"
#include <eigen3/Eigen/Dense>

Solution<6> IK_gen_6_dof(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin);

#endif // IK_GEN_6_DOF_H_
