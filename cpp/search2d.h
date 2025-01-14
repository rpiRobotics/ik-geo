#ifndef SEARCH2D_H_
#define SEARCH2D_H_

#include "utils.h"
#include <nlopt.hpp>

template<int N>
std::vector<std::tuple<double, double, unsigned>> search_2d(std::function<Eigen::Matrix<double, N, 1>(double, double)> f, double min0, double min1, double max0, double max1, unsigned n) {
    const unsigned N_MAX_MINIMA = 1000;
    const double MIN_THRESHOLD = 1e-1;

    double delta0 = (max0 - min0) / n;
    double delta1 = (max1 - min1) / n;

    std::vector<double> x0_vals(n, 0);
    std::vector<double> x1_vals(n, 0);

    Eigen::Matrix<double, N, 1> zero;
    zero.setZero();
    std::vector<Eigen::Matrix<double, N, 1>> mesh(n * n, zero);

    auto minimum = [&mesh, n]() {
        double min = std::numeric_limits<double>::infinity();
        unsigned min_i = 0;
        unsigned min_j = 0;
        unsigned min_k = 0;

        for (unsigned i = 0; i < n; ++i) {
            for (unsigned j = 0; j < n; ++j) {
                for (unsigned k = 0; k < N; ++k) {
                    double x = mesh[i + j * n][k];

                    if (x < min) {
                        min = x;
                        min_i = i;
                        min_j = j;
                        min_k = k;
                    }
                }
            }
        }

        if (std::isfinite(min)) {
            return std::optional<std::tuple<unsigned, unsigned, unsigned>> {std::make_tuple(min_i, min_j, min_k)};
        }
        else {
            return std::optional<std::tuple<unsigned, unsigned, unsigned>> {};
        }
    };

    std::function<void(int, int, int)> clear_blob;

    clear_blob = [&mesh, n, &clear_blob](int i, int j, int k) {
        i = modulo(i, n);
        j = modulo(j, n);

        if (std::isnan(mesh[i + j * n][k])) {
            return;
        }

        mesh[i + j * n][k] = std::numeric_limits<double>::quiet_NaN();

        clear_blob(i + 1, j, k);
        clear_blob(i - 1, j, k);
        clear_blob(i, j + 1, k);
        clear_blob(i, j - 1, k);
    };

    for (unsigned i = 0; i < n; ++i) {
        x0_vals[i] = i * delta0 + min0;
        x1_vals[i] = i * delta1 + min1;
    }

    for (unsigned i = 0; i < n; ++i) {
        for (unsigned j = 0; j < n; ++j) {
            Eigen::Matrix<double, N, 1> v = f(x0_vals[i], x1_vals[j]);

            for (unsigned i = 0; i < N; ++i) {
                if (v[i] > MIN_THRESHOLD) v[i] = std::numeric_limits<double>::quiet_NaN();
            }

            mesh[i + j * n] = v;
        }
    }

    std::vector<std::tuple<double, double, unsigned>> minima;
    std::vector<std::tuple<double, double, unsigned>> optimized;

    for (unsigned i = 0; i < N_MAX_MINIMA; ++i) {
        std::optional<std::tuple<unsigned, unsigned, unsigned>> min = minimum();

        if (min) {
            std::tuple<unsigned, unsigned, unsigned> min_location = *min;
            unsigned i = std::get<0>(min_location);
            unsigned j = std::get<1>(min_location);
            unsigned k = std::get<2>(min_location);
            minima.push_back(std::make_tuple(x0_vals[i], x1_vals[j], k));
            clear_blob(i, j, k);
        }
        else {
            break;
        }
    }

    if (minima.size() >= N_MAX_MINIMA) {
        std::cerr << "Too many minima found." << std::endl;
        exit(-1);
    }

    for (auto const &[x0, x1, k] : minima) {
        ProblemParams<N> params = { f, k };
        nlopt::opt solver(nlopt::LN_NELDERMEAD, 2);
        solver.set_min_objective(problem_objective<N>, &params);
        solver.set_stopval(1e-6);
        solver.set_maxeval(200 * N);

        std::vector<double> guess { x0, x1 };
        double value;

        nlopt::result result = solver.optimize(guess, value);

        if (result > 0) {
            optimized.push_back(std::make_tuple(guess[0], guess[1], k));
        }
        else {
            std::cerr << "warning: nlopt failed to optimize\n";
        }
    }

    return optimized;
}

#endif // SEARCH2D_H_