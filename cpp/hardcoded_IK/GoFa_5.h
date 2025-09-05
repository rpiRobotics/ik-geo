#ifndef GOFA_5_H_
#define GOFA_5_H_

#include "../utils.h"
#include "../IK/IK_2_parallel_2_intersecting.h"

Solution<6> GoFa_5_IK(const Eigen::Matrix3d &R_06, const Eigen::Vector3d &p_06, const Kinematics<6, 7> &kin);
struct GoFa_5_Setup {
    SEWConv m_sew;
    Kinematics<6, 7> m_kin;
    Eigen::Matrix3d m_R_6T;
    Eigen::Matrix3d m_R_0T;
    Eigen::Vector3d m_p_0T;
    double m_psi;
    Solution<6> m_sol;
    Eigen::Matrix<double, 6, 1> m_q_given;

    GoFa_5_Setup();
    GoFa_5_Setup(const Eigen::Matrix<double, 6, 1>& q_given);
    GoFa_5_Setup(const std::string& csv_line);
    void initialize_kinematics();
    void run();
    double error();
    double error_to_q_given() const;
    void debug() const;

};

#endif // GOFA_5_H_