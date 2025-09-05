
#ifndef IK_2_PARALLEL_2_INTERSECTING_H_
#define IK_2_PARALLEL_2_INTERSECTING_H_

// h_2 h_3 parallel
// h_4 and h_5 intersecting
// e.g. ABB GoFa

#include "../utils.h"

Solution<6> IK_2_parallel_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin);

#endif // IK_2_PARALLEL_2_INTERSECTING_H_