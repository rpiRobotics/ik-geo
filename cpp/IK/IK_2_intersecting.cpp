#include <iostream>
#include "IK_2_intersecting.h"
#include "../subproblems/sp.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

Eigen::Matrix4d q_partial_given_q4(double q4, const Kinematics &kin,
                                   const Eigen::Vector3d &p_16) {
    Eigen::Matrix4d q_partial;
    q_partial.setConstant(NAN);

    Eigen::Vector3d p_35_3 = kin.P.col(3) + rot(kin.H.col(3), q4) * kin.P.col(4);

    std::vector<double> t1;
    std::vector<double> t2;
    std::vector<double> t3;
    IKS::sp5_run(-kin.P.col(1), p_16, kin.P.col(2), p_35_3, -kin.H.col(0),
                 kin.H.col(1), kin.H.col(2), t1, t2, t3);

    for (unsigned i = 0; i < t1.size(); ++i) {
        Eigen::Vector4d col;
        col << t1[i], t2[i], t3[i], q4;
        q_partial.col(i) = col;
    }

    return q_partial;
}

Solution IK_2_intersecting(const Eigen::Matrix<double, 3, 3> &R_06,
                           const Eigen::Vector3d &p_0T, const Kinematics &kin) {
    Solution soln;

    Eigen::Vector3d p_16 = p_0T - kin.P.col(0) - R_06 * kin.P.col(6);

    auto alignment_error_given_q4 = [&R_06, &kin, &p_16](double q4) {
        Eigen::Vector4d error;
        error.fill(INFINITY);

        Eigen::Vector3d p_35_3 =
            kin.P.col(3) + rot(kin.H.col(3), q4) * kin.P.col(4);

        std::vector<double> t1;
        std::vector<double> t2;
        std::vector<double> t3;
        IKS::sp5_run(-kin.P.col(1), p_16, kin.P.col(2), p_35_3, -kin.H.col(0),
                     kin.H.col(1), kin.H.col(2), t1, t2, t3);

        for (unsigned i = 0; i < t1.size(); ++i) {
            Eigen::Matrix3d r_04 = rot(kin.H.col(0), t1[0]) *
                rot(kin.H.col(1), t2[0]) *
                rot(kin.H.col(2), t3[0]) * rot(kin.H.col(3), q4);

            error[i] =
                (kin.H.col(4).transpose() * r_04.transpose() * R_06 * kin.H.col(5) -
                 kin.H.col(4).transpose() * kin.H.col(5))(0);
        }

        return error;
    };

    std::vector<std::pair<double, unsigned>> q4_vec =
        search_1d<4>(alignment_error_given_q4, -M_PI, M_PI, 200);

    for (std::pair<double, unsigned> zero : q4_vec) {
        double q4 = zero.first;
        unsigned i = zero.second;

        Eigen::Vector4d q_partial = q_partial_given_q4(q4, kin, p_16).col(i);

        Eigen::Matrix3d r_04 =
            rot(kin.H.col(0), q_partial[0]) * rot(kin.H.col(1), q_partial[1]) *
            rot(kin.H.col(2), q_partial[2]) * rot(kin.H.col(3), q_partial[3]);

        double q5;
        bool q5_is_ls =
            IKS::sp1_run(kin.H.col(5), (r_04.transpose() * R_06 * kin.H.col(5)),
                         kin.H.col(4), q5);

        double q6;
        bool q6_is_ls =
            IKS::sp1_run(kin.H.col(4), (R_06.transpose() * r_04 * kin.H.col(4)),
                         -kin.H.col(5), q6);

        Eigen::Matrix<double, 6, 1> q;
        q << q_partial[0], q_partial[1], q_partial[2], q_partial[3], q5, q6;
        soln.q.push_back(q);
        soln.is_ls.push_back(q5_is_ls || q6_is_ls);
    }

    return soln;
}
