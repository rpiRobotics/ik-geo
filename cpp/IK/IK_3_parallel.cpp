#include "IK_3_parallel.h"
#include "../subproblems/sp.h"

Solution IK_3_parallel(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kinematics& kin) {
    Solution soln;

    Eigen::Vector3d p16 = p_0T - kin.P.col(0) - R_0T * kin.P.col(6);

    Eigen::Matrix<double, 3, 4> h_sp;
    Eigen::Matrix<double, 3, 4> k_sp;
    Eigen::Matrix<double, 3, 4> p_sp;

    h_sp << kin.H.col(1), kin.H.col(1), kin.H.col(1), kin.H.col(1);
    k_sp << -kin.H.col(0), kin.H.col(4), -kin.H.col(0), kin.H.col(4);
    p_sp << p16, -kin.P.col(5), R_0T * kin.H.col(5), -kin.H.col(5);

    double d1 = kin.H.col(1).transpose() * (kin.P.col(2) + kin.P.col(3) + kin.P.col(4) + kin.P.col(1));
    double d2 = 0.0;

    std::vector<double> theta1;
    std::vector<double> theta5;

    IKS::sp6_temp(p_sp, k_sp, h_sp, d1, d2, theta1, theta5);

    for (unsigned i = 0; i < theta1.size(); ++i) {
        double q1 = theta1[i];
        double q5 = theta5[i];

        Eigen::Matrix<double, 3, 3> r_01 = rot(kin.H.col(0), q1);
        Eigen::Matrix<double, 3, 3> r_45 = rot(kin.H.col(4), q5);

        double theta14;
        double q6;
        bool theta14_is_ls = IKS::sp1_run(r_45 * kin.H.col(5), r_01.transpose() * R_0T * kin.H.col(5), kin.H.col(1), theta14);
        bool q6_is_ls = IKS::sp1_run(r_45.transpose() * kin.H.col(1), R_0T.transpose() * r_01 * kin.H.col(1), -kin.H.col(5), q6);

        Eigen::Matrix<double, 3, 3> r_14 = rot(kin.H.col(1), theta14);
        Eigen::Vector3d d_inner = r_01.transpose() * p16 - kin.P.col(1) - r_14 * r_45 * kin.P.col(5) - r_14 * kin.P.col(4);
        double d = d_inner.norm();

        std::vector<double> theta3;
        bool theta3_is_ls = IKS::sp3_run(-kin.P.col(3), kin.P.col(2), kin.H.col(1), d, theta3);

        for (double q3 : theta3) {
            double q2;
            bool q2_is_ls = IKS::sp1_run(kin.P.col(2) + rot(kin.H.col(1), q3) * kin.P.col(3), d_inner, kin.H.col(1), q2);
            double q4 = wrap_to_pi(theta14 - q2 - q3);

            Eigen::Matrix<double, 6, 1> q;
            q << q1, q2, q3, q4, q5, q6;
            soln.q.push_back(q);
            soln.is_ls.push_back(theta14_is_ls || theta3_is_ls || q2_is_ls || q6_is_ls);
        }
    }

    return soln;
}
