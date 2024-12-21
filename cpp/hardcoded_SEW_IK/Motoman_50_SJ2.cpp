#include "../SEW_IK/IK_R_2R_R_3R_SJ2.h"
#include "Motoman_50_SJ2.h"

Motoman_50_SJ2_Setup::Motoman_50_SJ2_Setup() : sew(rand_vec()) {
    Eigen::Vector3d ex;
    Eigen::Vector3d ey;
    Eigen::Vector3d ez;
    Eigen::Vector3d zv;

    q_given = rand_angle(7);

    ex << 1, 0, 0;
    ey << 0, 1, 0;
    ez << 0, 0, 1;
    zv.setZero();


    double d1 = 0.540;
    double a1 = 0.145;
    double d3 = 0.875;
    double d5 = 0.610;
    double dT = 0.350;

    kin.P << d1*ez, a1*ex, zv, d3*ez, d5*ez, zv, zv, dT*ez;
    kin.H << ez, -ey, ez, -ey, ez, -ey, ez;

    sew = SEWConv(rot(ey, -M_PI/4) * ez);

    std::vector<unsigned> inter = {1, 3, 4};
    std::vector<Eigen::Vector3d> p_sew = kin.forward_kinematics_inter(q_given, inter, R, T);

    psi = sew.fwd_kin(p_sew[0], p_sew[1], p_sew[2]);
}

void Motoman_50_SJ2_Setup::run() {
    sol = IK_R_2R_R_3R_SJ2(R, T, sew, psi, kin);
}

double Motoman_50_SJ2_Setup::error() {
    double error = INFINITY;
    std::vector<unsigned> inter = {1, 3, 4};

    for (Eigen::Matrix<double, 7, 1> q : sol.q) {
        Eigen::Matrix3d R_t;
        Eigen::Vector3d T_t;
        std::vector<Eigen::Vector3d> p_sew_t = kin.forward_kinematics_inter(q, inter, R_t, T_t);
        double psi_t = sew.fwd_kin(p_sew_t[0], p_sew_t[1], p_sew_t[2]);
        double error_i = (R_t - R).norm() + (T_t - T).norm() + wrap_to_pi(psi_t - psi);

        if (error_i < error) error = error_i;
    }

    return error;
}

double Motoman_50_SJ2_Setup::error_to_q_given() const {
    double error = INFINITY;

    for (const Eigen::Matrix<double, 7, 1>& q : sol.q) {
        double error_i = wrap_to_pi(q - q_given).norm();
        if (error_i < error) error = error_i;
    }

    return error;
}

void Motoman_50_SJ2_Setup::debug() const {
    std::cout << "q_given: " << q_given.transpose() << std::endl;
    std::cout << "kin.H: " << std::endl << kin.H << std::endl;
    std::cout << "kin.P: " << std::endl << kin.P << std::endl;
    std::cout << "R: " << std::endl << R << std::endl;
    std::cout << "T: " << T.transpose() << std::endl;
    std::cout << "psi: " << psi << std::endl;
    std::cout << "sol.q: " << std::endl;
    for (const auto& q : sol.q) {
        std::cout << q.transpose() << std::endl;
    }
    std::cout << "sol.is_ls: " << std::endl;
    for (const auto& is_ls : sol.is_ls) {
        std::cout << is_ls << std::endl;
    }
}
