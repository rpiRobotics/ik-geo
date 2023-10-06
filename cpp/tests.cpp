#include <fstream>
#include <iomanip>
#include <iostream>
#include "IK_correctness.h"
#include "IK/IK_spherical_2_parallel.h"
#include "IK/IK_spherical_2_intersecting.h"
#include "utils.h"
#include <chrono>

void test(const char *data_path, Solution (*ik)(const Eigen::Matrix<double, 3, 3> &, const Eigen::Vector3d &, const Kinematics &)) {
    std::ifstream data_file(data_path);
    std::string line;
    std::vector<Setup> setups;

    std::getline(data_file, line);

    std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1);

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

    std::cout << "Avg min error = " << error_sum / counted << std::endl
              << counted << "/" << setups.size() << std::endl
              << "Batch time = " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / setups.size() << " ns" << std::endl;
}

int main() {
    std::cout << "Spherical Two Parallel" << std::endl;
    test("data/IkSphericalTwoParallel.csv", IK_spherical_2_parallel);
    std::cout << "Spherical Two Intersecting" << std::endl;
    test("data/IkSphericalTwoIntersecting.csv", IK_spherical_2_intersecting);
}
