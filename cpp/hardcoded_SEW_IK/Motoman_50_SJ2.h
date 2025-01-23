#ifndef MOTOMAN_50_SJ2_H_
#define MOTOMAN_50_SJ2_H_

#include "../utils.h"

Solution<7> MM50_IK(const Eigen::Matrix3d &R_07, const Eigen::Vector3d &p_0T, const SEWConv &SEW_class, double psi, const Kinematics<7, 8> &kin);
Eigen::Matrix4d MM50_IK_calculate_partial_q(const Kinematics<7, 8> &kin, const Eigen::Vector3d &p_1W, double q1);
struct Motoman_50_SJ2_Setup {
    SEWConv m_sew;
    Kinematics<7, 8> m_kin;
    Eigen::Matrix3d m_R;
    Eigen::Vector3d m_T;
    double m_psi;
    Solution<7> m_sol;
    Eigen::Matrix<double, 7, 1> m_q_given;

    Motoman_50_SJ2_Setup();
    Motoman_50_SJ2_Setup(const std::string& csv_line);
    void initialize_kinematics();
    void run();
    double error();
    double error_to_q_given() const;
    void debug() const;

};

#endif // MOTOMAN_50_SJ2_H_
