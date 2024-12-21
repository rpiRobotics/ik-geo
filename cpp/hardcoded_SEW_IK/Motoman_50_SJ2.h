#ifndef MOTOMAN_50_SJ2_H_
#define MOTOMAN_50_SJ2_H_

#include "../utils.h"

struct Motoman_50_SJ2_Setup {
    SEWConv sew;
    Kinematics<7, 8> kin;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    double psi;
    Solution<7> sol;
    Eigen::Matrix<double, 7, 1> q_given;

    Motoman_50_SJ2_Setup();
    void run();
    double error();
    double error_to_q_given() const;
    void debug() const;
};


#endif // MOTOMAN_50_SJ2_H_
