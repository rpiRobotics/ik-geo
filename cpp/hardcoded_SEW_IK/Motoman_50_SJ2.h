#ifndef MOTOMAN_50_SJ2_H_
#define MOTOMAN_50_SJ2_H_

#include "../utils.h"

class Motoman_50_SJ2_Setup {
    private:
        SEWConv sew;
        Kinematics<7, 8> kin;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        double psi;

        Solution<7> sol;

    public:
        Motoman_50_SJ2_Setup();

        void run();
        double error();
        void debug();
};


#endif // MOTOMAN_50_SJ2_H_
