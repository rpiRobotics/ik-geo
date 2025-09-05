#include "GoFa_5.h"
#include "../subproblems/sp.h"
#include <sstream>

GoFa_5_Setup::GoFa_5_Setup() {
    initialize_kinematics();
    m_q_given = rand_angle(6);

    // Run FK
    Eigen::Matrix3d R_06;
    m_kin.forward_kinematics(m_q_given, R_06, m_p_0T);
    m_R_0T = R_06*m_R_6T;
}

GoFa_5_Setup::GoFa_5_Setup(const Eigen::Matrix<double, 6, 1>& q_given) {
    initialize_kinematics();
    m_q_given = q_given;

    // Run FK
    Eigen::Matrix3d R_06;
    m_kin.forward_kinematics(m_q_given, R_06, m_p_0T);
    m_R_0T = R_06*m_R_6T;
}

void GoFa_5_Setup::initialize_kinematics() {
    Eigen::Vector3d ex(1, 0, 0);
    Eigen::Vector3d ey(0, 1, 0);
    Eigen::Vector3d ez(0, 0, 1);
    Eigen::Vector3d zv = Eigen::Vector3d::Zero();

    // MATLAB reference
    // kin.H = [ez ey ey ex ey ex];
    // kin.P = [zv 0.265*ez 0.444*ez 0.11*ez 0.47*ex 0.08*ez 0.101*ex];
    // kin.joint_type = [0 0 0 0 0 0];
    //
    // R_6T = round(rot(ey, pi/2));
    //
    // q_min = deg2rad([-180 -180 -225 -180 -180 -270]');
    // q_max = deg2rad([ 180  180   85  180  180  270]');

    m_kin.H << ez, ey, ey, ex, ey, ex;
    m_kin.P << zv, 0.265 * ez, 0.444 * ez, 0.11 * ez, 0.47 * ex, 0.08 * ez, 0.101 * ex;
    m_R_6T = rot(ey, M_PI/2).unaryExpr([](double x){ return std::round(x); }); // Round each entry
}

void GoFa_5_Setup::run() {
    Eigen::Matrix3d R_06 = m_R_0T * m_R_6T.transpose();
    m_sol = IK_2_parallel_2_intersecting(R_06, m_p_0T, m_kin);
}

double GoFa_5_Setup::error_to_q_given() const {
    double error = INFINITY;

    for (const Eigen::Matrix<double, 6, 1>& q : m_sol.q) {
        double error_i = wrap_to_pi(q - m_q_given).norm();
        if (error_i < error) error = error_i;
    }

    return error;
}