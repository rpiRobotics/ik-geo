/*
To compile with cpp as the working directory:
g++ -Wall -O3 tests_and_demos/example_spherical_two_parallel.cpp IK/IK_spherical_2_parallel.cpp subproblems/sp.cpp utils.cpp IK_correctness.cpp -I. -o example_spherical_two_parallel

Reference MATLAB code for IRB_6640 which uses IK_spherical_2_parallel:
zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
kin.H = [ez ey ey ex ey ex];
kin.P = [zv, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, zv, zv, 0.2*ex];
kin.joint_type = zeros([6 1]);

*/

#include "IK/IK_spherical_2_parallel.h"
#include "utils.h"
#include "IK_correctness.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

int main() {
    // Define the Kinematics structure
    Kinematics<6, 7> kin;
    kin.H << Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 1, 0),
             Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(1, 0, 0);
    kin.P << Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.32, 0, 0.78), Eigen::Vector3d(0, 0, 1.075),
             Eigen::Vector3d(1.1425, 0, 0.2), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
             Eigen::Vector3d(0.2, 0, 0);

    // Define the end effector pose
    Eigen::Matrix<double, 3, 3> R_0T = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Vector3d p_0T(0.5, 0.5, 0.5);

    // Solve the inverse kinematics
    Solution<6> soln = IK_spherical_2_parallel(R_0T, p_0T, kin);

    // Print the solutions and perform forward kinematics
    for (size_t i = 0; i < soln.q.size(); ++i) {
        std::cout << "Solution " << i + 1 << ": " << soln.q[i].transpose() << std::endl;
        std::cout << "Is least squares: " << soln.is_ls[i] << std::endl;

        // Perform forward kinematics
        Eigen::Matrix<double, 3, 3> R;
        Eigen::Vector3d p;
        kin.forward_kinematics(soln.q[i], R, p);
        std::cout << "Forward Kinematics Position: " << p.transpose() << std::endl;
        std::cout << "Forward Kinematics Rotation Matrix:\n" << R << std::endl;
    }

    // Demonstrate using the Setup class
    std::string raw_data = 
        "0,0,1," // h_1
        "0,1,0,"
        "0,1,0,"
        "1,0,0,"
        "0,1,0,"
        "1,0,0," // h_6
        "0,0,0," // p_01
        "0.32,0,0.78," 
        "0,0,1.075,"
        "1.1425,0,0.2,"
        "0,0,0,"
        "0,0,0,"
        "0.2,0,0," // p_67
        "1,0,0," // R_0T
        "0,1,0,"
        "0,0,1,"
        "0.5,0.5,0.5"; // p_0T
    Setup setup(raw_data);

    // Debug the Setup class
    std::cout << "Debugging the Setup class:" << std::endl;
    setup.debug();

    // Solve the IK problem using the Setup class
    setup.solve(IK_spherical_2_parallel);

    // Print the solutions from the Setup class
    std::cout << "Solutions from Setup class:" << std::endl;
    for (size_t i = 0; i < setup.q.size(); ++i) {
        std::cout << "Solution " << i + 1 << ": " << setup.q[i].transpose() << std::endl;
        std::cout << "Is least squares: " << setup.is_ls[i] << std::endl;

        // Perform forward kinematics using the Setup class
        Eigen::Matrix<double, 3, 3> R;
        Eigen::Vector3d p;
        setup.kin.forward_kinematics(setup.q[i], R, p);
        std::cout << "Forward Kinematics Position: " << p.transpose() << std::endl;
        std::cout << "Forward Kinematics Rotation Matrix:\n" << R << std::endl;
    }

    // Calculate and print the error
    double error = setup.error();
    std::cout << "Error: " << error << std::endl;

    return 0;
}