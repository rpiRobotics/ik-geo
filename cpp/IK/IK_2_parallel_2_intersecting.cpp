// h_2 h_3 parallel
// h_4 and h_5 intersecting
// e.g. ABB GoFa

#include "IK_2_parallel_2_intersecting.h"
#include "../subproblems/sp.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "../search.h"


Solution<6> IK_2_parallel_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kinematics<6, 7>& kin){
    Solution<6> sol;

    Eigen::Vector3d p_06 = p_0T - kin.P.col(0) - R_06 * kin.P.col(6);

    Eigen::Vector4d Q4 = Eigen::Vector4d::Constant(NAN);
    Eigen::Vector4d Q6 = Eigen::Vector4d::Constant(NAN);
    Eigen::Vector4d T13 = Eigen::Vector4d::Constant(NAN);

    auto error = [&Q4, &Q6, &T13, &R_06, &p_06, &kin](double q1) mutable {
        Q4.setConstant(NAN);
        Q6.setConstant(NAN);
        T13.setConstant(NAN);
        Eigen::Matrix3d R_01 = rot(kin.H.col(0), q1);
        unsigned i_sol = 0;

        Eigen::Vector4d error_vec = Eigen::Vector4d::Constant(NAN);

        // Solve for q6 using subproblem 4
        std::vector<double> t6; t6.push_back(NAN);  // TODO: Refactor sp4_run so it can take empty return vector
        bool t6_is_ls = IKS::sp4_run( // Careful: Argument order is different for SP4 between MATLAB and C++ versions
            kin.P.col(5), // p
            -kin.H.col(5), // k
            (kin.H.col(1).transpose() * R_01.transpose() * R_06).transpose(), // h
            kin.H.col(1).transpose() * (R_01.transpose() * p_06 - kin.P.col(1) - kin.P.col(2) - kin.P.col(3)), //d
            t6);

        if (t6_is_ls) {
            return error_vec;
        }

        for (double q6 : t6){
            Eigen::Matrix3d R_56 = rot(kin.H.col(5), q6);
            // Solve for q4 using subproblem 4
            std::vector<double> t4; t4.push_back(NAN);
            bool t4_is_ls = IKS::sp4_run(
                kin.H.col(4), // p
                kin.H.col(3), // k
                kin.H.col(1), // h
                kin.H.col(1).transpose() * R_01.transpose() * R_06 * R_56.transpose() * kin.H.col(4), // d
                t4);

            if (t4_is_ls) {
                i_sol += 2;
                continue;
            }

            for (double q4 : t4){
                Eigen::Matrix3d R_34 = rot(kin.H.col(3), q4);
                // Solve for theta_13 using Subproblem 1
                double t13;
                // bool t13_is_ls =
                IKS::sp1_run(
                    R_34 * kin.H.col(4),
                    R_01.transpose() * R_06 * R_56.transpose() * kin.H.col(4),
                    kin.H.col(1),
                    t13
                );
                Eigen::Matrix3d R_13 = rot(kin.H.col(1), t13);

                Q4(i_sol) = q4;
                Q6(i_sol) = q6;
                T13(i_sol) = t13;
                error_vec(i_sol) =
                    (R_01.transpose() * p_06 - kin.P.col(1) - R_13 * kin.P.col(3) - R_13 * R_34 * kin.P.col(4) - R_01.transpose() * R_06 * R_56.transpose() * kin.P.col(5)).norm()
                    - kin.P.col(2).norm();
                i_sol++;
            }
        }

        return error_vec;
    };


    std::vector<std::pair<double, unsigned>> zeros = search_1d<4>(error, -M_PI, M_PI, 10e3);

    for (std::pair<double, unsigned> zero : zeros) {
        double q1 = zero.first;
        unsigned i = zero.second;

        error(q1);
        Eigen::Matrix3d R_01 = rot(kin.H.col(0), q1);
        Eigen::Matrix3d R_34 = rot(kin.H.col(3), Q4(i));
        Eigen::Matrix3d R_56 = rot(kin.H.col(5), Q6(i));
        Eigen::Matrix3d R_13 = rot(kin.H.col(1), T13(i));

        // Solve for q2 and q5 each with Subproblem 1

        double q2;
        bool q2_is_ls = IKS::sp1_run(
            kin.P.col(2),
            R_01.transpose() * p_06 - kin.P.col(1) - R_13 * kin.P.col(3) - R_13 * R_34 * kin.P.col(4) - R_01.transpose() * R_06 * R_56.transpose() * kin.P.col(5),
            kin.H.col(1),
            q2);

        double q5;
        bool q5_is_ls = IKS::sp1_run(
            R_34.transpose() * kin.H.col(1),
            R_56 * R_06.transpose() * R_01 * kin.H.col(1),
            -kin.H.col(4),
            q5);

        double q3 = wrap_to_pi(T13(i) - q2);

        Eigen::Matrix<double, 6, 1> q_i;
        q_i << q1, q2, q3, Q4(i), q5, Q6(i);
        sol.q.push_back(q_i);
        sol.is_ls.push_back(q2_is_ls || q5_is_ls);
    }

    return sol;
}