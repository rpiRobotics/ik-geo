#ifndef IK_CORRECTNESS_H_
#define IK_CORRECTNESS_H_

#include <eigen3/Eigen/Dense>
#include <vector>
#include "utils.h"

struct Setup {
    Kinematics<6, 7> kin;
    Eigen::Matrix<double, 3, 3> r;
    Eigen::Matrix<double, 3, 1> t;

    std::vector<Eigen::Matrix<double, 6, 1>> q;
    std::vector<bool> is_ls;

    Setup(const std::string &raw);

    void solve(Solution<6> (*ik)(const Eigen::Matrix<double, 3, 3> &, const Eigen::Vector3d &, const Kinematics<6, 7> &));
    double error();
    void debug();
};

#endif // IK_CORRECTNESS_H_
