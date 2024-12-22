#include "Motoman_50_SJ2.h"
#include "../subproblems/sp.h"
#include "../search.h"
#include <sstream>

Motoman_50_SJ2_Setup::Motoman_50_SJ2_Setup() {
    initialize_kinematics();

    q_given = rand_angle(7);

    std::vector<unsigned> inter = {1, 3, 4};
    std::vector<Eigen::Vector3d> p_sew = kin.forward_kinematics_inter(q_given, inter, R, T);
    psi = sew.fwd_kin(p_sew[0], p_sew[1], p_sew[2]);
}

Motoman_50_SJ2_Setup::Motoman_50_SJ2_Setup(const std::string& csv_line) {
    initialize_kinematics();

    std::vector<std::string> tokens;
    std::stringstream ss(csv_line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }
    
    R << std::stod(tokens[0]), std::stod(tokens[3]), std::stod(tokens[6]),
         std::stod(tokens[1]), std::stod(tokens[4]), std::stod(tokens[7]),
         std::stod(tokens[2]), std::stod(tokens[5]), std::stod(tokens[8]);
    
    T << std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]);

    psi = std::stod(tokens[12]);

    q_given << std::stod(tokens[13]), std::stod(tokens[14]), std::stod(tokens[15]),
               std::stod(tokens[16]), std::stod(tokens[17]), std::stod(tokens[18]),
               std::stod(tokens[19]);
}

void Motoman_50_SJ2_Setup::initialize_kinematics() {
    Eigen::Vector3d ex(1, 0, 0);
    Eigen::Vector3d ey(0, 1, 0);
    Eigen::Vector3d ez(0, 0, 1);
    Eigen::Vector3d zv = Eigen::Vector3d::Zero();

    double d1 = 0.540;
    double a1 = 0.145;
    double d3 = 0.875;
    double d5 = 0.610;
    double dT = 0.350;

    kin.P << d1*ez, a1*ex, zv, d3*ez, d5*ez, zv, zv, dT*ez;
    kin.H << ez, -ey, ez, -ey, ez, -ey, ez;

    sew = SEWConv(rot(ey, -M_PI/4) * ez);   
}

void Motoman_50_SJ2_Setup::run() {
    sol = MM50_IK(R, T, sew, psi, kin);
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


Solution<7> Motoman_50_SJ2_Setup::MM50_IK(const Eigen::Matrix3d &R_07, const Eigen::Vector3d &p_0T, const SEWConv &SEW_class, double psi, const Kinematics<7, 8> &kin) {
    Solution<7> sol;

    Eigen::Vector3d W = p_0T - R_07 * kin.P.col(7);
    Eigen::Vector3d p_1W = W - kin.P.col(0);

    Eigen::Matrix4d partial_q;

    auto error = [&partial_q, &kin, &p_1W, &SEW_class, psi](double q1) mutable {
        Eigen::Vector2d psi_vec; // Only size 2 rather than 4 since we're skipping half of t4
        unsigned i_sol = 0;

        psi_vec.fill(NAN);
        partial_q.fill(NAN);

        Eigen::Vector3d p_1S = rot(kin.H.col(0), q1) * kin.P.col(1);
        Eigen::Vector3d p_SW = p_1W - p_1S;

        // instead of looping over t4, only use the first element of t4 because the error will be always duplicated for the MM50
        std::vector<double> t4;
        IKS::sp3_run(kin.P.col(4), -kin.P.col(3), kin.H.col(3), p_SW.norm(), t4);

        double q4 = t4[0];
        std::vector<double> t2;
        std::vector<double> t3;

        t2.push_back(0);
        t3.push_back(0);

        bool t23_is_LS = IKS::sp2_run(rot(kin.H.col(0), q1).transpose() * p_SW, kin.P.col(3) + rot(kin.H.col(3), q4) * kin.P.col(4), -kin.H.col(1), kin.H.col(2), t2, t3);
        if (t2.size() == 1) {
            // duplicate solns for t2 and t3
            t2.push_back(t2[0]);
            t3.push_back(t3[0]);
        }

        for (unsigned i_23 = 0; i_23 < t2.size(); ++i_23) {
            double q2 = t2[i_23];
            double q3 = t3[i_23];

            Eigen::Vector3d p_1E = p_1S + rot(kin.H.col(0), q1) * rot(kin.H.col(1), q2) * rot(kin.H.col(2), q3) * kin.P.col(3);
            double psi_i = SEW_class.fwd_kin(p_1S, p_1E, p_1W);
            Eigen::Vector4d q_i;

            psi_vec[i_sol] = wrap_to_pi(psi_i - psi);
            q_i << q1, q2, q3, q4;
            partial_q.col(i_sol) = q_i;
            i_sol += 1;
        }
        

        return psi_vec;
    };

    std::vector<std::pair<double, unsigned>> zeros = search_1d_no_cross_thresh<2>(error, -M_PI, M_PI, 500); // Size 2 rather than 4 because we're skipping half of t4

    // Each zero representing q1 needs to be duplicated because we only used the first element of t4
    // q1 stays the same, but the solution number is incremented by 2
    std::vector<std::pair<double, unsigned>> duplicated_zeros;
    for (const auto& zero : zeros) {
        duplicated_zeros.push_back(zero);
    }
    for (const auto& zero : zeros) {
        duplicated_zeros.push_back(std::make_pair(zero.first, zero.second + 2));
    }

    for (std::pair<double, unsigned> zero : duplicated_zeros) {
        double q1 = zero.first;
        unsigned i = zero.second;


        partial_q =  calculate_partial_q(p_1W, SEW_class, psi, q1); // Calculate partial_q using all of q1 values
        Eigen::Vector4d q_partial_col = partial_q.col(i);

        Eigen::Matrix3d R_01 = rot(kin.H.col(0), q_partial_col[0]);
        Eigen::Matrix3d R_12 = rot(kin.H.col(1), q_partial_col[1]);
        Eigen::Matrix3d R_23 = rot(kin.H.col(2), q_partial_col[2]);
        Eigen::Matrix3d R_34 = rot(kin.H.col(3), q_partial_col[3]);
        Eigen::Matrix3d R_04 = R_01 * R_12 * R_23 * R_34;

        std::vector<double> t5;
        std::vector<double> t6;

        t5.push_back(0);
        t6.push_back(0);

        bool t56_is_ls = IKS::sp2_run(R_04.transpose()*R_07*kin.H.col(6), kin.H.col(6), -kin.H.col(4), kin.H.col(5), t5, t6);

        for (unsigned i_56 = 0; i_56 < t5.size(); ++i_56) {
            double q5 = t5[i_56];
            double q6 = t6[i_56];

            Eigen::Matrix3d R_45 = rot(kin.H.col(4), q5);
            Eigen::Matrix3d R_56 = rot(kin.H.col(5), q6);
            Eigen::Vector3d p = kin.H.col(5);
            Eigen::Matrix3d R_06 = R_04 * R_45 * R_56;

            double q7;
            bool q7_is_ls = IKS::sp1_run(p, R_06.transpose() * R_07 * p, kin.H.col(6), q7);

            Eigen::Matrix<double, 7, 1> q_i;
            q_i << q_partial_col, q5, q6, q7;
            sol.q.push_back(q_i);
            sol.is_ls.push_back(t56_is_ls || q7_is_ls);
        }
    }

    return sol;
}

// Unlike the error function, loop through all of t1 to calculate the partial_q matrix
Eigen::Matrix4d Motoman_50_SJ2_Setup::calculate_partial_q(const Eigen::Vector3d &p_1W, const SEWConv &SEW_class, double psi, double q1) {
    Eigen::Matrix4d partial_q;
    unsigned i_sol = 0;

    partial_q.fill(NAN);

    Eigen::Vector3d p_1S = rot(kin.H.col(0), q1) * kin.P.col(1);
    Eigen::Vector3d p_SW = p_1W - p_1S;

    std::vector<double> t4;
    IKS::sp3_run(kin.P.col(4), -kin.P.col(3), kin.H.col(3), p_SW.norm(), t4);
    if (t4.size() < 2) {
        t4.push_back(t4[0]); // duplicate if LS
    }

    for (double q4 : t4) {
        std::vector<double> t2;
        std::vector<double> t3;

        t2.push_back(0);
        t3.push_back(0);

        IKS::sp2_run(rot(kin.H.col(0), q1).transpose() * p_SW, kin.P.col(3) + rot(kin.H.col(3),q4)*kin.P.col(4), -kin.H.col(1), kin.H.col(2), t2, t3);
        if (t2.size() < 2) {
            // duplicate solns for t2 and t3
            t2.push_back(t2[0]);
            t3.push_back(t3[0]);
        }

        for (unsigned i_23 = 0; i_23 < t2.size(); ++i_23) {
            double q2 = t2[i_23];
            double q3 = t3[i_23];

            Eigen::Vector4d q_i;

            q_i << q1, q2, q3, q4;
            partial_q.col(i_sol) = q_i;
            i_sol += 1;
        }
    }

    return partial_q;
};