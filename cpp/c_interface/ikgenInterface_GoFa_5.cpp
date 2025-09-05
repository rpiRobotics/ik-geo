// To compile:
// g++ -shared -fPIC -o libikgenInterface_GoFa_5.so -O3 c_interface/ikgenInterface_GoFa_5.cpp IK/IK_2_parallel_2_intersecting.cpp subproblems/sp.cpp utils.cpp -I. -std=c++17 -Wall -Werror -Wextra -Wshadow -Wfatal-errors


// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "ikgenInterface.h"

#include "../utils.h"
#include "../IK/IK_2_parallel_2_intersecting.h"

extern "C" {

ikgen_JointPos* computeInverseKinematics(
		const ikgen_EEPos& eePos,
		const ikgen_EEOri& eeOri,
		int& num_solutions)
{
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond q;
    q.x() = eeOri.x;
    q.y() = eeOri.y;
    q.z() = eeOri.z;
    q.w() = eeOri.w;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_0T = q.normalized().toRotationMatrix();

    // Assemble position as a vector
    Eigen::Vector3d p_0T(eePos.x, eePos.y, eePos.z);

    // Define kinematic parameters for the ABB GoFa 5kg
    // This is the same code as in GoFa_5.cpp
    Eigen::Vector3d ex = Eigen::Vector3d::UnitX();
    Eigen::Vector3d ey = Eigen::Vector3d::UnitY();
    Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d zv = Eigen::Vector3d::Zero();

    Kinematics<6, 7> kin;
    kin.H << ez, ey, ey, ex, ey, ex;
    kin.P << zv, 0.265 * ez, 0.444 * ez, 0.11 * ez, 0.47 * ex, 0.08 * ez, 0.101 * ex;
    Eigen::Matrix3d R_6T = rot(ey, M_PI/2).unaryExpr([](double x){ return std::round(x); }); // Round each entry

    // Run IK
    Eigen::Matrix3d R_06 = R_0T * R_6T.transpose();
    Solution<6> sol = IK_2_parallel_2_intersecting(R_06, p_0T, kin);

    // sol also contains the is_ls vector, which we ignore here

    num_solutions = static_cast<int>(sol.q.size());
    
    if (num_solutions == 0)
        return nullptr;

    // Allocate array for solutions
    // Don't forget to free this memory in the calling code
    ikgen_JointPos* jointPos = new ikgen_JointPos[num_solutions];

    // Copy solutions to the output array
    for (int i = 0; i < num_solutions; ++i) {
        const Eigen::Matrix<double, 6, 1>& q_i = sol.q[i];
        jointPos[i].x1 = q_i(0);
        jointPos[i].x2 = q_i(1);
        jointPos[i].x3 = q_i(2);
        jointPos[i].x4 = q_i(3);
        jointPos[i].x5 = q_i(4);
        jointPos[i].x6 = q_i(5);
    }

    return jointPos;

}
} // extern "C"
