#ifndef SEARCH_H_
#define SEARCH_H_

#include "utils.h"

template <int N>
double find_min(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned i) {
    const unsigned ITERATIONS = 100;
    const double EPSILON = 1e-12;

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

        if (fabs(x_right - x_left) < EPSILON || fabs(y_right - y_left) < EPSILON) {
            break;
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

/*
Similar to search_1d_no_cross_thresh but also use minimization and maximization

Rather than just checking for zero crossings between two points,
we also check for triplets of points where
    the middle point is larger than (smaller than) the other two and
    all three points are smaller than (larger than) zero
If such a triplet is found, we iterate to find the maxium (minimum)
    bracketed by the closest two points among the triplet
If the maximum (minimum) is above (below) 0, then we have found 2 zero crossings
And we iterate on those zero crossings to find the function zeros

TODO: Make this work with the +-pi wraparound
*/
template<int N>
std::vector<std::pair<double, unsigned>> search_1d_min_max(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned initial_samples) {
    double delta = (right - left) / (double)(initial_samples-1);

    std::vector<std::pair<double, unsigned>> zeros;

    double x[3] = {left - delta, left, left + delta};
    Eigen::Matrix<double, N, 1> v[3] = {f(x[1]), f(x[1]), f(x[2])}; // first entry will be overwritten

    for (unsigned n = 2; n < initial_samples; ++n) {
        // Move the buffer
        x[0] = x[1];
        x[1] = x[2];
        x[2] = left + n * delta;
        v[0] = v[1];
        v[1] = v[2];
        v[2] = f(x[2]);


        for (unsigned i = 0; i < N; ++i) {
            double y[3] = {v[0](i), v[1](i), v[2](i)};

            if ((y[1] < 0.0) != (y[0] < 0.0)) { // if sign changed between y[0] and y[1]
            double z;
            if (find_zero(f, x[0], x[1], i, z)) {
                zeros.push_back(std::make_pair(z, i));
            }
            } else if (y[1] < y[0] && y[1] < y[2] && y[0] > 0.0 && y[1] > 0.0 && y[2] > 0.0) { // triangle pointing down
            double min_x = find_min<N>(f, x[0], x[2], i);
            if (f(min_x)(i) < 0.0) {
                double z1, z2;
                if (min_x < x[1]) { // min falls between x[0] and x[1]
                    if (find_zero(f, x[0], min_x, i, z1)) {
                        zeros.push_back(std::make_pair(z1, i));
                    }
                    if (find_zero(f, min_x, x[1], i, z2)) {
                        zeros.push_back(std::make_pair(z2, i));
                    }
                } else { // min falls between x[1] and x[2]
                    if (find_zero(f, x[1], min_x, i, z1)) {
                        zeros.push_back(std::make_pair(z1, i));
                    }
                    if (find_zero(f, min_x, x[2], i, z2)) {
                        zeros.push_back(std::make_pair(z2, i));
                    }
                }
            }
            } else if (y[1] > y[0] && y[1] > y[2] && y[0] < 0.0 && y[1] < 0.0 && y[2] < 0.0) { // triangle pointing up
            double max_x = find_max<N>(f, x[0], x[2], i);
            if (f(max_x)(i) > 0.0) {
                double z1, z2;
                if (max_x < x[1]) { // max falls between x[0] and x[1]
                    if (find_zero(f, x[0], max_x, i, z1)) {
                        zeros.push_back(std::make_pair(z1, i));
                    }
                    if (find_zero(f, max_x, x[1], i, z2)) {
                        zeros.push_back(std::make_pair(z2, i));
                    }
                } else { // max falls between x[1] and x[2]
                    if (find_zero(f, x[1], max_x, i, z1)) {
                        zeros.push_back(std::make_pair(z1, i));
                    }
                    if (find_zero(f, max_x, x[2], i, z2)) {
                        zeros.push_back(std::make_pair(z2, i));
                    }
                }
            }
            }
        }
    }    
    // Don't forget to check last pair of points
    for (unsigned i = 0; i < N; ++i) {   
        if ((v[1](i) < 0.0) != (v[2](i) < 0.0)) { // if sign changed between v[1] and v[2]
            double z;
            if (find_zero(f, x[1], x[2], i, z)) {
                zeros.push_back(std::make_pair(z, i));
            }
        }
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

#endif // SEARCH_H_
