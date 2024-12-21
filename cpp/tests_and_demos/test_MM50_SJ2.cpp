#include "hardcoded_SEW_IK/Motoman_50_SJ2.h"
#include "utils.h"
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

int main() {
    // test_MM50_random();
    // test_MM50_csv();
    // test_MM50_csv_file();

    test_MM50_csv_file_bulk();

    return 0;
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