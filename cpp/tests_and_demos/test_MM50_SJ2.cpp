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

    std::cout << "----------------------------------------" << std::endl;

    // Now test importing a CSV line
    std::string csv_line = "-0.057996647518894,0.000440446470090519,0.998316680659737,-0.438620746249246,0.898299606595998,-0.0258777462161887,-0.89679887925684,-0.439383229989928,-0.0519051766907153,0.115426042890603,0.0931067009559481,-0.130789826252996,2.79249835640583,0.294565272647012,2.87460022633501,2.92098081384033,-2.15128045457413,2.95682165202307,2.87246465212821,-0.0918875090716771";
    Motoman_50_SJ2_Setup setup2(csv_line);

    // Run the inverse kinematics solver
    setup2.run();

    // Print the solutions
    std::cout << "Solutions:" << std::endl;
    for (size_t i = 0; i < setup2.sol.q.size(); ++i) {
        std::cout << "Solution " << i + 1 << ": " << setup2.sol.q[i].transpose() << std::endl;
    }

    // Calculate and print the error
    double error2 = setup2.error();
    std::cout << "Error: " << error2 << std::endl;

    // Calculate and print the error to the given q
    double error_to_q_given2 = setup2.error_to_q_given();
    std::cout << "Error to q_given: " << error_to_q_given2 << std::endl;

    setup2.debug();

    return 0;
}