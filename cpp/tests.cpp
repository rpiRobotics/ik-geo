#include <fstream>
#include <iomanip>
#include <iostream>
#include "IK_correctness.h"
#include "IK/IK_spherical_2_parallel.h"
#include "IK/IK_spherical_2_intersecting.h"
#include "IK/IK_spherical.h"
#include "IK/IK_3_parallel_2_intersecting.h"
#include "IK/IK_3_parallel.h"
#include "IK/IK_2_parallel.h"
#include "utils.h"
#include <chrono>

void test(const char *data_path, Solution (*ik)(const Eigen::Matrix<double, 3, 3> &, const Eigen::Vector3d &, const Kinematics &)) {
    std::ifstream data_file(data_path);
    std::string line;
    std::vector<Setup> setups;

    std::getline(data_file, line);

    while (std::getline(data_file, line)) {
        setups.emplace_back(line);
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for (Setup &setup : setups) {
        setup.solve(ik);
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    double error_sum = 0.0;
    unsigned counted = 0;

    for (Setup &setup : setups) {
        double error = setup.error();
        if (error < 1e-6) {
            error_sum += error;
            ++counted;
        }
    }

    std::cout << "\tAvg min error = " << error_sum / counted << std::endl
              << '\t' << counted << "/" << setups.size() << std::endl
              << "\tBatch time = " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / setups.size() << " ns" << std::endl;
}

int main() {
    std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1);

    std::cout << "Spherical Two Parallel" << std::endl;
    test("data/IkSphericalTwoParallel.csv", IK_spherical_2_parallel);

    std::cout << "Spherical Two Intersecting" << std::endl;
    test("data/IkSphericalTwoIntersecting.csv", IK_spherical_2_intersecting);

    std::cout << "Spherical" << std::endl;
    test("data/IkSpherical.csv", IK_spherical);

    std::cout << "Three Parallel Two Intersecting" << std::endl;
    test("data/IkThreeParallelTwoIntersecting.csv", IK_3_parallel_2_intersecting);

    std::cout << "Three Parallel" << std::endl;
    test("data/IkThreeParallel.csv", IK_3_parallel);

    std::cout << "Two Parallel" << std::endl;
    test("data/IkTwoParallel.csv", IK_2_parallel);
}
