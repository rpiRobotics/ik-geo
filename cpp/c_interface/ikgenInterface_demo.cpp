// To compile:
// g++ -shared -fPIC -o libikgenInterface_demo.so -O3 c_interface/ikgenInterface_demo.cpp -I. -std=c++17 -Wall  -Wextra -Wshadow -Wfatal-errors
// -Werror may also be used


// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "ikgenInterface.h"

extern "C" {

ikgen_JointPos* computeInverseKinematics(
		const ikgen_EEPos& eePos,
		const ikgen_EEOri& eeOri,
		int& num_solutions)
{

    // ikfast::IkSolutionList<double> solutions;

    // double t_ptr[] = {eePos.x, eePos.y, eePos.z};

    Eigen::Quaterniond q;
    q.x() = eeOri.x;
    q.y() = eeOri.y;
    q.z() = eeOri.z;
    q.w() = eeOri.w;

    // Convert quaternion to rotation matrix
    // Data of rotation matrix must be stored in row-major form
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = q.normalized().toRotationMatrix();
    // double* R_ptr = R.data();


    // if (num_solutions == 0)
    // {
    // 	// no solution found
    // 	jointPos = nullptr;
    //     return jointPos;

    // } else
    // {
    // 	jointPos = new ikgen_JointPos[num_solutions];
    // }

    // double theta_ptr[6]; // for reading out solutions

    // for (size_t i=0; i<num_solutions; i++)
    // {
    // 	// nullptr = no free joints
    // 	solutions.GetSolution(i).GetSolution(theta_ptr, nullptr);

    //     jointPos[i].x1 = theta_ptr[0];
    //     jointPos[i].x2 = theta_ptr[1];
    //     jointPos[i].x3 = theta_ptr[2];
    //     jointPos[i].x4 = theta_ptr[3];
    //     jointPos[i].x5 = theta_ptr[4];
    //     jointPos[i].x6 = theta_ptr[5];

    // }

    // Generate a simulated solution
    // two solutions: one of all zeros, one of [0.1 0.2 ... 0.6] rad
    num_solutions = 2;
    ikgen_JointPos* jointPos = new ikgen_JointPos[num_solutions];

    // First solution: all zeros
    jointPos[0].x1 = 0.0;
    jointPos[0].x2 = 0.0;
    jointPos[0].x3 = 0.0;
    jointPos[0].x4 = 0.0;
    jointPos[0].x5 = 0.0;
    jointPos[0].x6 = 0.0;

    // Second solution: [0.1, 0.2, ..., 0.6] radians
    jointPos[1].x1 = 0.1;
    jointPos[1].x2 = 0.2;
    jointPos[1].x3 = 0.3;
    jointPos[1].x4 = 0.4;
    jointPos[1].x5 = 0.5;
    jointPos[1].x6 = 0.6;

    return jointPos;

}
} // extern "C"
