#ifndef IK_R_2R_R_3R_SJ2_H_
#define IK_R_2R_R_3R_SJ2_H_

#include "../utils.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

class IK_R_2R_R_3R_SJ2_Setup : Setup {
    private:
        SEWConv sew;
        Kinematics<7, 8> kin;
        Eigen::Matrix3d R;
        Eigen::Matrix3d T;
        double psi;

        Solution<7> sol;

    public:
        IK_R_2R_R_3R_SJ2_Setup() : sew(rand_normal_vec()) {
            Eigen::Vector3d zeros;
            Eigen::Matrix<double, 7, 1> q;
            zeros.fill(0);
            for (unsigned i = 0; i < 7; ++i) q[i] = rand_angle();

            kin.P << rand_vec(), rand_vec(), zeros, rand_vec(), rand_vec(), zeros, zeros, rand_vec();
            kin.H = rand_normal_vec(7);



        }
};


Solution<7> IK_R_2R_R_3R_SJ2(const Eigen::Matrix3d &R_07, const Eigen::Vector3d &p_0T, const SEWConv &SEW_class, double psi, const Kinematics<7, 8> &kin);

#endif // IK_R_2R_R_3R_SJ2_H_
