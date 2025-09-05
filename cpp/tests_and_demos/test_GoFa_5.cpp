// To compile:
// g++ -Wall -Werror -Wextra -Wshadow -O3 tests_and_demos/test_GoFa_5.cpp hardcoded_IK/GoFa_5.cpp IK/IK_2_parallel_2_intersecting.cpp subproblems/sp.cpp utils.cpp -I. -std=c++17 -o test_GoFa_5 -Wfatal-errors


#include "hardcoded_IK/GoFa_5.h"
#include "utils.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>

void test_GoFa_5_FK();
void test_GoFa_5_given_q();
void test_GoFa_5_random();
void test_GoFa_5_bulk_random();

int main() {
    test_GoFa_5_FK();
    // test_GoFa_5_given_q();
    // test_GoFa_5_random();
    // test_GoFa_5_bulk_random();
    return 0;
}

void test_GoFa_5_FK(){
    GoFa_5_Setup setup;
    Eigen::Matrix3d R_06;
    Eigen::Vector3d p_0T;
    Eigen::Matrix<double, 6, 1> q_0 = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> q_1;
    q_1 << 10, 20, 30, 40, 50, 60; q_1 *= (M_PI / 180.0);

    std::cout << "Test Case 1: Zero pose";
    std::cout << "\nq: " << q_0.transpose() << std::endl;
    setup.m_kin.forward_kinematics(q_0, R_06, p_0T);
    std::cout << "R_0T: " << std::endl << R_06*setup.m_R_6T << std::endl;
    std::cout << "p_0T: " << p_0T.transpose() << std::endl;

    std::cout << "Expected R_0T: " << std::endl;
    std::cout <<  rot(Eigen::Vector3d::UnitY(), (M_PI / 180.0)*(90)) << std::endl;
    std::cout << "Expected p_0T: " << std::endl;
    std::cout << Eigen::Vector3d(0.571, 0, 0.899).transpose() << std::endl;

    std::cout << std::endl;
    std::cout << "Test Case 2";
    std::cout << "\nq: " << q_1.transpose() << std::endl;
    setup.m_kin.forward_kinematics(q_1, R_06, p_0T);
    std::cout << "R_0T: " << std::endl << R_06*setup.m_R_6T << std::endl;
    std::cout << "p_0T: " << p_0T.transpose() << std::endl;

    std::cout << "Expected R_0T: " << std::endl;
    std::cout <<    rot(Eigen::Vector3d::UnitZ(), (M_PI / 180.0)*(100.55))
                  * rot(Eigen::Vector3d::UnitY(), (M_PI / 180.0)*(-29.54))
                  * rot(Eigen::Vector3d::UnitX(), (M_PI / 180.0)*(-178.19)) << std::endl;
    std::cout << "Expected p_0T: " << std::endl;
    std::cout << Eigen::Vector3d(0.59205, 0.12133, 0.28343).transpose() << std::endl;

}

void test_GoFa_5_given_q(){
    Eigen::Matrix<double, 6, 1> q;
    // q << 10, 20, 30, 40, 50, 60; q *= (M_PI / 180.0);
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

    GoFa_5_Setup setup(q);

    // Print FK results
    std::cout << "R_0T: " << std::endl << setup.m_R_0T << std::endl;
    std::cout << "p_0T: " << setup.m_p_0T.transpose() << std::endl;
    // Also print R_0T as a quaternion
    std::cout << "R_0T (quaternion): " << std::endl << Eigen::Quaterniond(setup.m_R_0T) << std::endl;

    // Compute IK
    setup.run();

    // Print the solutions
    std::cout << "Solutions:" << std::endl;
    for (size_t i = 0; i < setup.m_sol.q.size(); ++i) {
        std::cout << "Solution " << i + 1 << ": " << setup.m_sol.q[i].transpose() << std::endl;
    }

    // Calculate and print the error to the given q
    double error_to_q_given = setup.error_to_q_given();
    std::cout << "Error to q_given: " << error_to_q_given << std::endl;
}

void test_GoFa_5_random(){
    GoFa_5_Setup setup;

    // Compute IK
    setup.run();

    // Print the solutions
    std::cout << "Solutions:" << std::endl;
    for (size_t i = 0; i < setup.m_sol.q.size(); ++i) {
        std::cout << "Solution " << i + 1 << ": " << setup.m_sol.q[i].transpose() << std::endl;
    }

    // Calculate and print the error to the given q
    double error_to_q_given = setup.error_to_q_given();
    std::cout << "Error to q_given: " << error_to_q_given << std::endl;
}

void test_GoFa_5_bulk_random() {
    int N = 10e3;
    std::vector<double> errors;
    for (int i = 0; i < N; ++i) {
        GoFa_5_Setup setup;
        setup.run();

        // calculate and save error to vector
        double error_to_q_given = setup.error_to_q_given();
        errors.push_back(error_to_q_given);
    }

    // Print all errors
    // std::cout << "All errors to q_given:" << std::endl;
    // for (const auto& error : errors) {
    //     std::cout << error << std::endl;
    // }

    // Save results to CSV file
    std::ofstream file("errors.csv");
    for (const auto& error : errors) {
        file << error << std::endl;
    }
    file.close();
}