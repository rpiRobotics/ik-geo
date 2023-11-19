#include "IK_correctness.h"
#include <iostream>

Setup::Setup(const std::string &raw) {
    std::vector<double> data;
    std::stringstream stream(raw);

    while (stream.good()) {
        std::string x;
        getline(stream, x, ',');
        data.push_back(std::stod(x));
    }

    for (int i = 0; i < 18; ++i) {
        kin.H(i % 3, i / 3) = data[i];
    }

    for (int i = 0; i < 21; ++i) {
        kin.P(i % 3, i / 3) = data[i + 18];
    }

    r << data[39], data[40], data[41],
         data[42], data[43], data[44],
         data[45], data[46], data[47];

    t << data[48],
         data[49],
         data[50];
}

double Setup::error() {
    double min_error = std::numeric_limits<double>::infinity();

    for (Eigen::Matrix<double, 6, 1> q_i : q) {
        Eigen::Matrix<double, 3, 3> r_t;
        Eigen::Matrix<double, 3, 1> t_t;

        kin.forward_kinematics(q_i, r_t, t_t);
        double error = (r_t - r).norm() + (t_t - t).norm();

        min_error = std::min(error, min_error);
    }

    return min_error;
}

void Setup::solve(Solution<6> (*ik)(const Eigen::Matrix<double, 3, 3> &, const Eigen::Vector3d &, const Kinematics<6, 7> &)) {
    Solution<6> s = ik(r, t, kin);
    q = s.q;
    is_ls = s.is_ls;
}

void Setup::debug() {
    std::cout << kin.H << std::endl
              << kin.P << std::endl
              << t << std::endl
              << r << std::endl;
}
