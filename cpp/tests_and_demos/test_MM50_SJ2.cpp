#include "hardcoded_SEW_IK/Motoman_50_SJ2.h"
#include "utils.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

/*
To compile:
g++ -Wall -O3 tests_and_demos/test_MM50_SJ2.cpp hardcoded_SEW_IK/Motoman_50_SJ2.cpp SEW_IK/IK_R_2R_R_3R_SJ2.cpp subproblems/sp.cpp utils.cpp -I. -std=c++17 -o test_MM50_SJ2
*/

int main() {
    // Create an instance of the Motoman_50_SJ2_Setup class
    Motoman_50_SJ2_Setup setup;

    // Run the inverse kinematics solver
    setup.run();

    // Print the solutions
    std::cout << "Solutions:" << std::endl;
    for (size_t i = 0; i < setup.sol.q.size(); ++i) {
        std::cout << "Solution " << i + 1 << ": " << setup.sol.q[i].transpose() << std::endl;
    }

    // Calculate and print the error
    double error = setup.error();
    std::cout << "Error: " << error << std::endl;

    // Calculate and print the error to the given q
    double error_to_q_given = setup.error_to_q_given();
    std::cout << "Error to q_given: " << error_to_q_given << std::endl;

    setup.debug();


    return 0;
}