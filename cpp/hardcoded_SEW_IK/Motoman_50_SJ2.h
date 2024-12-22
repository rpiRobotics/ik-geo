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
    Motoman_50_SJ2_Setup(const std::string& csv_line);
    void initialize_kinematics();
    void run();
    double error();
    double error_to_q_given() const;
    void debug() const;
    Solution<7> MM50_IK(const Eigen::Matrix3d &R_07, const Eigen::Vector3d &p_0T, const SEWConv &SEW_class, double psi, const Kinematics<7, 8> &kin);
    Eigen::Matrix4d calculate_partial_q(const Eigen::Vector3d &p_1W, const SEWConv &SEW_class, double psi, double q1);
};

#endif // MOTOMAN_50_SJ2_H_
