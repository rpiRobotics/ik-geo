#include "IK_R_2R_R_3R_SJ2.h"
#include "../subproblems/sp.h"

Solution<7> IK_R_2R_R_3R_SJ2(const Eigen::Matrix3d &R_07, const Eigen::Vector3d &p_0T, const SEWConv &SEW_class, double psi, const Kinematics<7, 8> &kin) {
    Solution<7> sol;

    Eigen::Vector3d W = p_0T - R_07 * kin.P.col(7);
    Eigen::Vector3d p_1W = W - kin.P.col(0);

    Eigen::Matrix4d partial_q;

    auto error = [&partial_q, &kin, &p_1W, &SEW_class, psi](double q1) mutable {
        Eigen::Vector4d psi_vec;
        unsigned i_sol = 0;

        psi_vec.fill(NAN);
        partial_q.fill(NAN);

        Eigen::Vector3d p_1S = rot(kin.H.col(0), q1) * kin.P.col(1);
        Eigen::Vector3d p_SW = p_1W - p_1S;

        std::vector<double> t4;
        bool t4_is_ls = IKS::sp3_run(kin.P.col(4), -kin.P.col(3), kin.H.col(3), p_SW.norm(), t4);

        if (t4_is_ls) return psi_vec;

        for (double q4 : t4) {
            std::vector<double> t2;
            std::vector<double> t3;

            bool t23_is_ls = IKS::sp2_run(rot(kin.H.col(0), q1).transpose() * p_SW, kin.P.col(3) + rot(kin.H.col(3),q4)*kin.P.col(4), -kin.H.col(1), kin.H.col(2), t2, t3);
            if (t23_is_ls) {
                i_sol += 2;
                continue;
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
        }

        return psi_vec;
    };

    std::vector<std::pair<double, unsigned>> zeros = search_1d<4>(error, -M_PI, M_PI, 200);

    for (std::pair<double, unsigned> zero : zeros) {
        double q1 = zero.first;
        unsigned i = zero.second;

        error(q1);
        Eigen::Vector4d q_partial_col = partial_q.col(i);

        Eigen::Vector3d R_01 = rot(kin.H.col(0), q_partial_col[0]);
        Eigen::Vector3d R_12 = rot(kin.H.col(1), q_partial_col[1]);
        Eigen::Vector3d R_23 = rot(kin.H.col(2), q_partial_col[2]);
        Eigen::Vector3d R_34 = rot(kin.H.col(3), q_partial_col[3]);
        Eigen::Vector3d R_04 = R_01 * R_12 * R_23 * R_34;

        std::vector<double> t5;
        std::vector<double> t6;
        bool t56_is_ls = IKS::sp2_run(R_04.transpose()*R_07*kin.H.col(6), kin.H.col(6), -kin.H.col(4), kin.H.col(5), t5, t6);

        for (unsigned i_56 = 0; i_56 < t5.size(); ++i_56) {
            double q5 = t5[i_56];
            double q6 = t6[i_56];

            Eigen::Vector3d R_45 = rot(kin.H.col(4), q5);
            Eigen::Vector3d R_56 = rot(kin.H.col(5), q6);
            Eigen::Vector3d p = kin.H.col(5);
            Eigen::Vector3d R_06 = R_04 * R_45 * R_56;

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
