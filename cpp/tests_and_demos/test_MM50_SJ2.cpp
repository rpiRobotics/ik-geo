#include "hardcoded_SEW_IK/Motoman_50_SJ2.h"
#include "utils.h"
#include "search.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>


/*
To compile:
g++ -Wall -O3 tests_and_demos/test_MM50_SJ2.cpp hardcoded_SEW_IK/Motoman_50_SJ2.cpp SEW_IK/IK_R_2R_R_3R_SJ2.cpp subproblems/sp.cpp utils.cpp -I. -std=c++17 -o test_MM50_SJ2
*/

void test_MM50_random();
void test_MM50_csv();
void test_MM50_csv_file();
void test_MM50_csv_file_bulk();
void test_MM50_single_line(int line_number);
void find_first_large_error();
void test_wrap_to_pi();
void test_min_max();

int main() {
    // test_MM50_random();
    // test_MM50_csv();
    // test_MM50_csv_file();
    // test_MM50_csv_file_bulk();

    // test_MM50_single_line(64);
    // find_first_large_error();
    // test_wrap_to_pi();
    test_min_max();
    return 0;
}
void test_min_max() {
    auto f = [](double x) {
        return Eigen::Matrix<double, 3, 1>(0, 0, x*x*x - 2*x*x + x); // x^3-2x^2+x
    };

    double min_result = find_min<3>(f, 0.4, 1.6, 2);
    std::cout << "Min found at x = " << min_result << std::endl;

    double max_result = find_max<3>(f, 0, 1, 2);
    std::cout << "Max found at x = " << max_result << std::endl;
}

void test_wrap_to_pi() {
    std::vector<double> angles = {0, M_PI, -M_PI, 2 * M_PI, -2 * M_PI, 3 * M_PI, -3 * M_PI, M_PI / 2, -M_PI / 2,
                                  4 * M_PI, -4 * M_PI, 5 * M_PI, -5 * M_PI, M_PI / 4, -M_PI / 4, 3 * M_PI / 2, -3 * M_PI / 2};
    std::vector<double> expected_results = {0, M_PI, -M_PI, 0, 0, M_PI, -M_PI, M_PI / 2, -M_PI / 2,
                                            0, 0, M_PI, -M_PI, M_PI / 4, -M_PI / 4, -M_PI / 2, M_PI / 2};

    bool all_tests_passed = true;
    for (size_t i = 0; i < angles.size(); ++i) {
        double wrapped_angle = wrap_to_pi(angles[i]);
        double expected_angle = expected_results[i];

        bool is_pi_case = (std::abs(wrapped_angle - M_PI) < 1e-6 && std::abs(expected_angle + M_PI) < 1e-6) ||
                          (std::abs(wrapped_angle + M_PI) < 1e-6 && std::abs(expected_angle - M_PI) < 1e-6);

        if (std::abs(wrapped_angle - expected_angle) > 1e-6 && !is_pi_case) {
            std::cout << "Test failed for angle " << angles[i] << ": expected " << expected_angle << ", got " << wrapped_angle << std::endl;
            all_tests_passed = false;
        }
    }

    if (all_tests_passed) {
        std::cout << "All wrap_to_pi tests passed!" << std::endl;
    }
}


void test_MM50_random() {
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
}

void test_MM50_csv() {
    std::string csv_line = "-0.057996647518894,0.000440446470090519,0.998316680659737,-0.438620746249246,0.898299606595998,-0.0258777462161887,-0.89679887925684,-0.439383229989928,-0.0519051766907153,0.115426042890603,0.0931067009559481,-0.130789826252996,-1.05587477252154,0.294565272647012,2.87460022633501,2.92098081384033,-2.15128045457413,2.95682165202307,2.87246465212821,-0.0918875090716771";
    Motoman_50_SJ2_Setup setup(csv_line);

    // Run the inverse kinematics solver
    setup.run();

    // Print the solutions
    std::cout << "Solutions:" << std::endl;
    for (size_t i = 0; i < setup.sol.q.size(); ++i) {
        std::cout << "Solution " << i + 1 << ": " << setup.sol.q[i].transpose() << std::endl;
    }

    // Calculate and print the error
    double error2 = setup.error();
    std::cout << "Error: " << error2 << std::endl;

    // Calculate and print the error to the given q
    double error_to_q_given2 = setup.error_to_q_given();
    std::cout << "Error to q_given: " << error_to_q_given2 << std::endl;

    setup.debug();
}

void test_MM50_csv_file() {
    std::ifstream file("data/hardcoded_IK_setup_MM50_SJ2.csv");

    std::vector<double> errors_to_q_given;

    std::string line;
    //skip the first line
    std::getline(file, line);

    int lines_to_read = -1; // Change this value to read more or fewer lines, set to -1 to read all lines
    int line_count = 0;
    while ((lines_to_read < 0 || line_count < lines_to_read) && std::getline(file, line)) {
        ++line_count;
        Motoman_50_SJ2_Setup setup(line);

        // print the line for debugging
        // std::cout << "Line " << line_count << ": " << line << std::endl;

        // Run the inverse kinematics solver
        setup.run();

        // // Print the solutions
        // std::cout << "Solutions:" << std::endl;
        // for (size_t i = 0; i < setup.sol.q.size(); ++i) {
        //     std::cout << "Solution " << i + 1 << ": " << setup.sol.q[i].transpose() << std::endl;
        // }

        // // Calculate and print the error
        // double error = setup.error();
        // std::cout << "Error: " << error << std::endl;

        // Calculate and print the error to the given q
        double error_to_q_given = setup.error_to_q_given();
        // std::cout << "Error to q_given: " << error_to_q_given << std::endl;
        errors_to_q_given.push_back(error_to_q_given);

        // setup.debug();
    }

    // Save the errors to a file
    std::ofstream error_file("data/errors_to_q_given.csv");
    for (const auto& error : errors_to_q_given) {
        error_file << error << "\n";
    }
    error_file.close();
}

void test_MM50_csv_file_bulk() {
    std::ifstream file("data/hardcoded_IK_setup_MM50_SJ2.csv");

    std::vector<Motoman_50_SJ2_Setup> setups;

    std::string line;
    // Skip the first line
    std::getline(file, line);

    int lines_to_read = -1; // Change this value to read more or fewer lines, set to -1 to read all lines
    int line_count = 0;
    while ((lines_to_read < 0 || line_count < lines_to_read) && std::getline(file, line)) {
        ++line_count;
        setups.emplace_back(line);
    }

    // Run the inverse kinematics solver for each setup and measure the time taken
    auto start = std::chrono::high_resolution_clock::now();
    for (auto& setup : setups) {
        setup.run();
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration = end - start;
    double avg_time = duration.count() / setups.size();
    std::cout << "Average time per pose: " << avg_time << " microseconds" << std::endl;

    // Calculate and save the errors to a file
    std::ofstream error_file("data/errors_to_q_given.csv");
    for (const auto& setup : setups) {
        double error_to_q_given = setup.error_to_q_given();
        error_file << error_to_q_given << "\n";
    }
    error_file.close();

    // Report the proportion of errors below 1e-1
    int count_below_threshold = 0;
    double threshold = 1e-1;
    for (const auto& setup : setups) {
        if (setup.error_to_q_given() < threshold) {
            ++count_below_threshold;
        }
    }
    double proportion_below_threshold = static_cast<double>(count_below_threshold) / setups.size();
    std::cout << "Proportion of errors below " << threshold << ": " << proportion_below_threshold << std::endl;
}

void test_MM50_single_line(int line_number) {
    std::ifstream file("data/hardcoded_IK_setup_MM50_SJ2.csv");

    std::string line;
    // Skip the first line
    std::getline(file, line);

    int current_line = 0;
    while (std::getline(file, line)) {
        ++current_line;
        if (current_line == line_number) {
            Motoman_50_SJ2_Setup setup(line);

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
            break;
        }
    }

    if (current_line != line_number) {
        std::cerr << "Line number " << line_number << " not found in the file." << std::endl;
    }
}

void find_first_large_error() {
    std::ifstream file("data/hardcoded_IK_setup_MM50_SJ2.csv");

    std::string line;
    // Skip the first line
    std::getline(file, line);

    int line_count = 0;
    double threshold = 1e-1;
    while (std::getline(file, line)) {
        ++line_count;
        Motoman_50_SJ2_Setup setup(line);

        // Run the inverse kinematics solver
        setup.run();

        // Calculate the error to the given q
        double error_to_q_given = setup.error_to_q_given();

        if (error_to_q_given > threshold) {
            std::cout << "First large error found at line " << line_count << ": " << error_to_q_given << std::endl;
            setup.debug();
            return;
        }
    }

    std::cout << "No error larger than " << threshold << " found in the file." << std::endl;
}