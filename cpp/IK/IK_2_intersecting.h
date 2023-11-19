#ifndef IK_2_INTERSECTING_H_
#define IK_2_INTERSECTING_H_

#include "../utils.h"

Solution<6> IK_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin);

#endif // IK_2_INTERSECTING_H_
