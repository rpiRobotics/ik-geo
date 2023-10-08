#include "IK_spherical.h"
#include "../subproblems/sp.h"
#include <vector>

Solution IK_spherical(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kinematics& kin) {
    Solution sol;

    Eigen::Vector3d p_16 = p_0T - kin.P.col(0) - R_0T * kin.P.col(6);

    std::vector<double> theta1;
    std::vector<double> theta2;
    std::vector<double> theta3;

    IKS::sp5_run(-kin.P.col(1), p_16, kin.P.col(2), kin.P.col(3), -kin.H.col(0), kin.H.col(1), kin.H.col(2), theta1, theta2, theta3);

    for (unsigned i = 0; i < theta1.size(); ++i) {
        double q1 = theta1[i];
        double q2 = theta2[i];
        double q3 = theta3[i];

        Eigen::Matrix<double, 3, 3> r_36 = rot(-kin.H.col(2), q3) * rot(-kin.H.col(1), q2) * rot(-kin.H.col(0), q1) * R_0T;

        std::vector<double> theta5;
        theta5.push_back(0);
        bool q5_is_ls = IKS::sp4_run(kin.H.col(5), kin.H.col(4), kin.H.col(3), kin.H.col(3).transpose() * r_36 * kin.H.col(5), theta5);

        for (double q5 : theta5) {
            double q4;
            double q6;

            bool q4_is_ls = IKS::sp1_run(rot(kin.H.col(4), q5) * kin.H.col(5), r_36 * kin.H.col(5), kin.H.col(3), q4);
            bool q6_is_ls = IKS::sp1_run(rot(-kin.H.col(4), q5) * kin.H.col(3), r_36.transpose() * kin.H.col(3), -kin.H.col(5), q6);

            Eigen::Matrix<double, 6, 1> q;
            q << q1, q2, q3, q4, q5, q6;
            sol.q.push_back(q);
            sol.is_ls.push_back(q5_is_ls || q4_is_ls || q6_is_ls);
        }
    }

    return sol;
}
