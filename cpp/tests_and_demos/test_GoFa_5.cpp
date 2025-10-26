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

void save_test_cases();
void test_missed_solutions_CSV();
void run_single_line_CSV_file(int line_number);
void test_spurious_solutions_CSV();

int main() {
    // test_GoFa_5_FK();
    // test_GoFa_5_given_q();
    // test_GoFa_5_random();
    // test_GoFa_5_bulk_random();
    
    // save_test_cases();
    test_missed_solutions_CSV();
    // run_single_line_CSV_file(2);
    // test_spurious_solutions_CSV();

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

// Save 10k random joint angles (6x1) to a CSV file
void save_test_cases() {
    int N = 10e3;
    std::ofstream file("gofa_test_cases.csv");
    for (int i = 0; i < N; ++i) {
        // use a deterministic (repeatable) RNG seed so tests are identical each run
        #include <random>
        Eigen::Matrix<double, 6, 1> q;
        static thread_local std::mt19937_64 rng(42ULL); // fixed seed for repeatability
        static thread_local std::uniform_real_distribution<double> dist(-M_PI, M_PI);
        for (int j = 0; j < 6; ++j) q(j) = dist(rng);
        for (int j = 0; j < q.size(); ++j) {
            file << q(j);
            if (j + 1 < q.size()) file << ",";
        }
        file << std::endl;
    }
    file.close();
}

// count number of missed solutions from a CSV file of test cases
// missed solution means no q in Q is close to q_given
// also return the index of the first missed solution and the q_given for that case
void test_missed_solutions_CSV() {
    std::ifstream file("gofa_test_cases.csv");
    std::string line;
    int line_count = 0;
    int missed_count = 0;
    double threshold = 1e-3;
    Eigen::Matrix<double, 6, 1> first_missed_q;
    int first_missed_index = -1;

    while (std::getline(file, line)) {
        ++line_count;
        GoFa_5_Setup setup(line);
        setup.run();
        double error_to_q_given = setup.error_to_q_given();
        if (error_to_q_given > threshold) {
            ++missed_count;
            if (first_missed_index == -1) {
                first_missed_index = line_count;
                first_missed_q = setup.m_q_given;
            }
        }
    }

    std::cout << "Total missed solutions: " << missed_count << " out of " << line_count << std::endl;
    if (first_missed_index != -1) {
        std::cout << "First missed solution at line " << first_missed_index << ": q_given = " << first_missed_q.transpose() << std::endl;
    }
}

void run_single_line_CSV_file(int line_number) {
    std::ifstream file("gofa_test_cases.csv");
    std::string line;
    int current_line = 0;

    while (std::getline(file, line)) {
        ++current_line;
        if (current_line == line_number) {
            GoFa_5_Setup setup(line);
            std::cout << "Running test case from line " << line_number << ": q_given = " << setup.m_q_given.transpose() << std::endl;
            setup.run();
            std::cout << "Solutions found:" << std::endl;
            for (size_t i = 0; i < setup.m_sol.q.size(); ++i) {
                std::cout << "Solution " << i + 1 << ": " << setup.m_sol.q[i].transpose() << std::endl;
            }
            double error_to_q_given = setup.error_to_q_given();
            std::cout << "Error to q_given: " << error_to_q_given << std::endl;
            return;
        }
    }

    std::cout << "Line number " << line_number << " not found in the file." << std::endl;
}

double R2angle(const Eigen::Matrix3d& R) {
    Eigen::Vector3d s;
    s << R(2,1) - R(1,2),
         R(0,2) - R(2,0),
         R(1,0) - R(0,1);
    s *= 0.5;

    const double c  = 0.5 * (R.trace() - 1.0);
    const double sn = s.norm();
    return std::atan2(sn, c);
}


// Test for spurious solutions from a CSV file of test cases
// For each q in Q, check if fwd_kin(q) is close to (R_0T, p_0T) given by q_given
void test_spurious_solutions_CSV() {
    std::ifstream file("gofa_test_cases.csv");
    if (!file) {
        std::cout << "Could not open gofa_test_cases.csv" << std::endl;
        return;
    }

    std::vector<double> errs_p;
    std::vector<double> errs_R;

    std::string line;
    int line_count = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        ++line_count;

        GoFa_5_Setup setup(line);
        setup.run();

        // For each IK solution compute FK and compare to the target task pose
        for (size_t i = 0; i < setup.m_sol.q.size(); ++i) {
            Eigen::Matrix3d R_06_sol;
            Eigen::Vector3d p_sol;
            setup.m_kin.forward_kinematics(setup.m_sol.q[i], R_06_sol, p_sol);

            Eigen::Matrix3d R_sol_0T = R_06_sol * setup.m_R_6T;
            double p_err = (p_sol - setup.m_p_0T).norm();

            Eigen::Matrix3d R_err = R_sol_0T * setup.m_R_0T.transpose();
            double angle_err = R2angle(R_err);

            errs_p.push_back(p_err);
            errs_R.push_back(angle_err);
        }
    }

    // Save the errors to a file
    std::ofstream error_file("gofa_task_space_errs.csv");
    for (size_t i = 0; i < errs_p.size(); ++i) {
        error_file << errs_p[i] << "," << errs_R[i] << "\n";
    }
    error_file.close();

    if (!errs_p.empty()) {
        double max_p = errs_p[0];
        double max_R = errs_R[0];
        for (size_t i = 1; i < errs_p.size(); ++i) {
            if (errs_p[i] > max_p) max_p = errs_p[i];
            if (errs_R[i] > max_R) max_R = errs_R[i];
        }
        std::cout << "Processed " << errs_p.size() << " solution poses from " << line_count << " test cases." << std::endl;
        std::cout << "Largest ||Delta p||: " << max_p << std::endl;
        std::cout << "Largest angle(Delta R): " << max_R << std::endl;
    } else {
        std::cout << "No solutions/errors recorded." << std::endl;
    }
}

