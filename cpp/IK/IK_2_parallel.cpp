//---------------------------------------------------------------//
// Name: IK_spherical_2_parallel.cpp
// Author: Runbin Chen
// Date: 04/01/2023
//---------------------------------------------------------------//

#include "IK_2_parallel.h"
#include "../subproblems/sp.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

Solution IK_2_parallel(const Eigen::Matrix<double, 3, 3> &R_06,
                       const Eigen::Vector3d &p_0T, const Kinematics &kin) {
    Solution soln;

    std::vector<double> t4;
    std::vector<double> t6;

    Eigen::Vector3d p_16 =
        p_0T - kin.P.block<3, 1>(0, 0) - R_06 * kin.P.block<3, 1>(0, 6);

    auto error_given_q1 =
        [kin, R_06, p_16, &t4, &t6](double q1) mutable {
            Eigen::Vector4d e;
            e << INFINITY, INFINITY, INFINITY, INFINITY;

            Eigen::Matrix3d R_01 = rot(kin.H.col(0), q1);
            Eigen::Vector3d h1 = (kin.H.col(1).transpose() * R_01.transpose() * R_06).transpose();
            Eigen::Vector3d h2 = kin.H.col(1);
            Eigen::Vector3d h3 = h1;
            Eigen::Vector3d h4 = h2; // no negative here
            Eigen::Matrix<double, 3, 4> sp_H, sp_K, sp_P;
            sp_H << h1, h2, h3, h4;
            sp_K << -kin.H.col(5), kin.H.col(3), -kin.H.col(5), kin.H.col(3);
            sp_P << kin.P.col(5), kin.P.col(4), kin.H.col(4), -kin.H.col(4);
            double d1 = kin.H.col(1).transpose() * (R_01.transpose() * p_16 - kin.P.col(1) - kin.P.col(2) - kin.P.col(3));
            double d2 = 0;
            t4.clear();
            t6.clear();
            IKS::sp6_run(sp_P, sp_K, sp_H, d1, d2, t6, t4);

            for (unsigned int i_46 = 0; i_46 < t4.size(); i_46++) {
                Eigen::Matrix<double, 3, 3> R_34 = rot(kin.H.col(3), t4[i_46]);
                Eigen::Matrix<double, 3, 3> R_56 = rot(kin.H.col(5), t6[i_46]);
                double t23;
                bool t23_is_LS = IKS::sp1_run(
                    R_34 * kin.H.col(4),
                    R_01.transpose() * R_06 * R_56.transpose() * kin.H.col(4),
                    kin.H.col(1), t23);
                Eigen::Matrix<double, 3, 3> R_13 = rot(kin.H.col(1), t23);

                e[i_46] = (R_01.transpose() * p_16 - kin.P.col(1) -
                           R_13 * kin.P.col(3) -
                           R_13 * R_34 * kin.P.col(4) -
                           R_01.transpose() * R_06 * R_56.transpose() *
                           kin.P.col(5)) .norm() - kin.P.col(2).norm();
            }

            return e;
        };

        std::vector<std::pair<double, unsigned>> zeros = search_1d<4>(error_given_q1, -M_PI, M_PI, 200);

    for (unsigned int i_q1 = 0; i_q1 < zeros.size(); i_q1++) {
        double q1 = zeros[i_q1].first;
        unsigned i1 = zeros[i_q1].second;
        error_given_q1(q1);
        double q6 = t6[i1];
        double q4 = t4[i1];

        Eigen::Matrix3d R_01 = rot(kin.H.col(0), q1);
        Eigen::Matrix3d R_34 = rot(kin.H.col(3), q4);
        Eigen::Matrix3d R_56 = rot(kin.H.col(5), q6);

        double t23;
        bool t23_is_ls = IKS::sp1_run(R_34 * kin.H.col(4), R_01.transpose() * R_06 * R_56.transpose() * kin.H.col(4), kin.H.col(1), t23);

        Eigen::Matrix3d R_13 = rot(kin.H.col(1), t23);

        double q2;
        bool q2_is_ls = IKS::sp1_run(kin.P.col(2), R_01.transpose() * p_16 - kin.P.col(1) - R_13 * kin.P.col(3) - R_13 * R_34 * kin.P.col(4) - R_01.transpose() * R_06 * R_56.transpose() * kin.P.col(5), kin.H.col(1), q2);

        double q5;
        bool q5_is_ls = IKS::sp1_run(R_34.transpose() * kin.H.col(1), R_56 * R_06.transpose() * R_01 * kin.H.col(1), -kin.H.col(4), q5);

        double q3 = wrap_to_pi(t23 - q2);

        Eigen::Matrix<double, 6, 1> q;
        q << q1, q2, q3, q4, q5, q6;
        soln.q.push_back(q);
        soln.is_ls.push_back(t23_is_ls || q2_is_ls || q5_is_ls);
    }

    return soln;
}
