#include <fstream>
#include <iomanip>
#include <iostream>
#include "IK_correctness.h"
#include "IK/IK_spherical_2_parallel.h"
#include "utils.h"

void test(const char *data_path, Solution (*ik)(const Eigen::Matrix<double, 3, 3> &, const Eigen::Vector3d &, const Kinematics &)) {
    std::ifstream data_file(data_path);
    std::string line;
    std::vector<Setup> setups;

    std::getline(data_file, line);

    std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1);

    while (std::getline(data_file, line)) {
        setups.emplace_back(line);
    }

    for (Setup &setup : setups) {
        setup.solve(ik);
    }

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
              << "Out of " << counted << std::endl;
}

int main() {
    test("data/IkSphericalTwoParallel.csv", IK_spherical_2_parallel);
}
