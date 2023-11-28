#ifndef IK_R_2R_R_3R_SJ2_H_
#define IK_R_2R_R_3R_SJ2_H_

#include "../utils.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

class IK_R_2R_R_3R_SJ2_Setup {
    private:
        SEWConv sew;
        Kinematics<7, 8> kin;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        double psi;

        Solution<7> sol;

    public:
        IK_R_2R_R_3R_SJ2_Setup();

        void run();
        double error();
        void debug();
};


Solution<7> IK_R_2R_R_3R_SJ2(const Eigen::Matrix3d &R_07, const Eigen::Vector3d &p_0T, const SEWConv &SEW_class, double psi, const Kinematics<7, 8> &kin);

#endif // IK_R_2R_R_3R_SJ2_H_
