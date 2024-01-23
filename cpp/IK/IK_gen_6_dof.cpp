#include "IK_gen_6_dof.h"
#include "../subproblems/sp.h"
#include "../search.h"

#define ASSERT(c)                                               \
    do {                                                        \
        if (!(c)) {                                             \
            std::cerr << "Assertion Failed: " #c << std::endl;  \
            exit(-1);                                           \
        }                                                       \
    } while (false)

bool q_given_q12_k(double q1, double q2, unsigned k, const Eigen::Vector3d &p16, const Eigen::Matrix3d &R_06, const Kinematics<6, 7> &kin, Eigen::Matrix<double, 6, 1> &q) {
        Eigen::Vector3d p63 = rot(-kin.H.col(1), q2) * (rot(-kin.H.col(0), q1) * p16 - kin.P.col(1)) - kin.P.col(2);
        Eigen::Vector3d p;
        p << 0, 0, 1;

        std::vector<double> t3;
        std::vector<double> t4;
        std::vector<double> t5;

        IKS::sp5_run(
            -kin.P.col(3),
            p63,
            kin.P.col(4),
            kin.P.col(5),
            -kin.H.col(2),
            kin.H.col(3),
            kin.H.col(4),
            t3, t4, t5
        );

        double q3 = t3[k];
        double q4 = t4[k];
        double q5 = t5[k];

        Eigen::Matrix3d R_05 = rot(kin.H.col(0), q1) * rot(kin.H.col(1), q2) *
                               rot(kin.H.col(2), q3) * rot(kin.H.col(3), q4) *
                               rot(kin.H.col(4), q5);

        double q6;
        bool q6_is_ls = IKS::sp1_run(p, R_05.transpose() * R_06 * p, kin.H.col(5), q6);

        q << q1, q2, q3, q4, q5, q6;

        return q6_is_ls;
}

Solution<6> IK_gen_6_dof(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin) {
    Solution<6> soln;

    Eigen::Vector3d p16 = p_0T - kin.P.col(0) - R_06 * kin.P.col(6);

    auto alignment_error_given_q12 = [&kin, &R_06, &p_0T, &p16](double q1, double q2) {
        Eigen::Vector4d error;
        error.fill(INFINITY);

        Eigen::Vector3d p63 = rot(-kin.H.col(1), q2) * (rot(-kin.H.col(0), q1) * p16 - kin.P.col(1)) - kin.P.col(2);

        std::vector<double> t3;
        std::vector<double> t4;
        std::vector<double> t5;

        IKS::sp5_run(
            -kin.P.col(3),
            p63,
            kin.P.col(4),
            kin.P.col(5),
            -kin.H.col(2),
            kin.H.col(3),
            kin.H.col(4),
            t3, t4, t5
        );

        for (unsigned i = 0; i < t3.size(); ++i) {
            double q3 = t3[i];
            double q4 = t4[i];
            double q5 = t5[i];

            Eigen::Matrix3d R_05 =
                rot(kin.H.col(0), q1) *
                rot(kin.H.col(1), q2) *
                rot(kin.H.col(2), q3) *
                rot(kin.H.col(3), q4) *
                rot(kin.H.col(4), q5);

            error[i] = (R_05 * kin.H.col(5) - R_06 * kin.H.col(5)).norm();
        }

        return error;
    };

    std::vector<std::tuple<double, double, unsigned>> minima = search_2d<4>(alignment_error_given_q12, -M_PI, -M_PI, M_PI, M_PI, 100);

    for (auto minimum : minima) {
        double x0 = std::get<0>(minimum);
        double x1 = std::get<1>(minimum);
        double k = std::get<2>(minimum);

        Eigen::Matrix<double, 6, 1> q;
        bool q_is_ls = q_given_q12_k(x0, x1, k, p16, R_06, kin, q);

        soln.q.push_back(q);
        soln.is_ls.push_back(q_is_ls);
    }

    return soln;
}
