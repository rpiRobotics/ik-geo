#ifndef SEARCH_H_
#define SEARCH_H_

#include "utils.h"

template <int N>
double find_min(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned i) {
    const unsigned ITERATIONS = 100;
    
    double x_left = left;
    double x_right = right;

    double y_left = f(x_left)(i);
    double y_right = f(x_right)(i);

    double best_x = x_left;
    double best_y = y_left;

    for (unsigned n = 0; n < ITERATIONS; ++n) {
        double x_mid = 0.5 * (x_left + x_right);
        double y_mid = f(x_mid)(i);

        if (y_mid < best_y) {
            best_x = x_mid;
            best_y = y_mid;
        }

        if (y_left < y_right) {
            x_right = x_mid;
            y_right = y_mid;
        } else {
            x_left = x_mid;
            y_left = y_mid;
        }
    }

    return best_x;
}

template <int N>
double find_max(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned i) {
    auto neg_f = [f](double x) {
        return -f(x);
    };
    return find_min<N>(neg_f, left, right, i);
}

template <int N>
bool find_zero(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned i, double &result) {
    const unsigned ITERATIONS = 100;
    const double EPSILON = 1e-12;
    const double EPSILON_X = 1e-12;

    double x_left = left;
    double x_right = right;

    double y_left = f(x_left)(i);
    double y_right = f(x_right)(i);

    double best_x = x_left;
    double best_y = fabs(y_left);

    for (unsigned n = 0; n < ITERATIONS; ++n) {
        double delta = y_right - y_left;

        if (fabs(delta) < EPSILON) {
            break;
        }

        if (fabs(x_right - x_left) < EPSILON_X) {
            break;
        }

        double x_0 = x_left - y_left * (x_right - x_left) / delta;
        double y_0 = f(x_0)(i);

        if (std::isinf(y_0)) {
            return false;
        }

        if (fabs(y_0) < best_y) {
            best_x = x_0;
            best_y = fabs(y_0);
        }

        if ((y_left < 0.0) == (y_0 < 0.0)) { // Same signs
            x_left = x_0;
            y_left = y_0;
        }
        else {
            x_right = x_0;
            y_right = y_0;
        }
    }

    result = best_x;
    return true;
}

template<int N>
std::vector<std::pair<double, unsigned>> search_1d(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned initial_samples) {
    const double CROSS_THRESHOLD = 0.1;

    double delta = (right - left) / (double)initial_samples;

    Eigen::Matrix<double, N, 1> last_v = f(left);
    double x = left + delta;

    std::vector<std::pair<double, unsigned>> zeros;

    for (unsigned n = 0; n < initial_samples; ++n) {
        Eigen::Matrix<double, N, 1> v = f(x);
        // std::cout << x << ' ' << v[0] << std::endl;

        for (unsigned i = 0; i < N; ++i) {
            double y = v(i);
            double last_y = last_v(i);

            if ((y < 0.0) != (last_y < 0.0) && fabs(y) < CROSS_THRESHOLD && fabs(last_y) < CROSS_THRESHOLD) {
                double z;

                if (find_zero(f, x - delta, x, i, z)) {
                    zeros.push_back(std::make_pair(z, i));
                }
            }
        }

        last_v = v;
        x += delta;
    }

    return zeros;
}

template<int N>
std::vector<std::pair<double, unsigned>> search_1d_no_cross_thresh(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned initial_samples) {
    double delta = (right - left) / (double)initial_samples;

    Eigen::Matrix<double, N, 1> last_v = f(left);
    double x = left + delta;

    std::vector<std::pair<double, unsigned>> zeros;

    for (unsigned n = 0; n < initial_samples; ++n) {
        Eigen::Matrix<double, N, 1> v = f(x);
        // std::cout << x << ' ' << v[0] << std::endl;

        for (unsigned i = 0; i < N; ++i) {
            double y = v(i);
            double last_y = last_v(i);

            if ((y < 0.0) != (last_y < 0.0)) { // if sign changed
                double z;

                if (find_zero(f, x - delta, x, i, z)) {
                    zeros.push_back(std::make_pair(z, i));
                }
            }
        }

        last_v = v;
        x += delta;
    }

    return zeros;
}

template<int N>
struct ProblemParams {
    std::function<Eigen::Matrix<double, N, 1>(double, double)> f;
    unsigned k;
};

template<int N>
double problem_objective(unsigned _n, const double *x, double *_grad, void *data) {
    (void) _n;
    (void) _grad;

    ProblemParams<N> *params = (ProblemParams<N> *) data;
    return params->f(x[0], x[1])[params->k];
}

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

#endif // SEARCH_H_
