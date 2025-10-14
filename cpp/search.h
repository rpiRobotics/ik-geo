#ifndef SEARCH_H_
#define SEARCH_H_

#include "utils.h"

// Golden section search
template <int N>
double find_min(std::function<Eigen::Matrix<double, N, 1>(double)> f,
                double left, double right, unsigned i){
    // if (i >= static_cast<unsigned>(N)) {
    //     throw std::out_of_range("component index i out of range");
    // }
    // if (!(left < right)) {
    //     throw std::invalid_argument("left must be < right");
    // }

    // Golden section constants
    const double phi = (1.0 + std::sqrt(5.0)) * 0.5;  // ~1.618
    const double invphi = 1.0 / phi;                  // ~0.618

    const double xtol = 1e-9;
    const double ftol = 1e-12;
    const int max_iter = 200;

    double a = left, b = right;

    double c = b - invphi * (b - a);
    double d = a + invphi * (b - a);
    double fc = f(c)(i);
    double fd = f(d)(i);

    for (int k = 0; k < max_iter; ++k) {
        if (fc < fd) {
            b = d;
            d = c; fd = fc;
            c = b - invphi * (b - a);
            fc = f(c)(i);
        } else {
            a = c;
            c = d; fc = fd;
            d = a + invphi * (b - a);
            fd = f(d)(i);
        }
        if (std::abs(b - a) <= xtol || std::abs(fd - fc) <= ftol) break;
    }
    return (fc < fd) ? c : d;
}

template <int N>
double find_max(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned i) {
    auto neg_f = [f](double x) {
        return -f(x);
    };
    return find_min<N>(neg_f, left, right, i);
}

// template <int N>
// bool find_zero(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned i, double &result) {
//     // For debugging: print the initial left and right values and their function values
//     std::cout << "Finding zero between " << left << " and " << right << std::endl;
//     std::cout << "f(" << left << ") = " << f(left)(i) << ", f(" << right << ") = " << f(right)(i) << std::endl;

//     const unsigned ITERATIONS = 500;
//     const double EPSILON = 1e-12; // Terminate if absolute function value is below this
//     const double EPSILON_TERMINAL = 1e-6; // Once we reach ITERATIONS, return true if absolute function value is below this
//     const double EPSILON_X = 1e-14;

//     double x_left = left;
//     double x_right = right;

//     double y_left = f(x_left)(i);
//     double y_right = f(x_right)(i);

//     double best_x = x_left;
//     double best_y = fabs(y_left);

//     for (unsigned n = 0; n < ITERATIONS; ++n) {
//         double delta = y_right - y_left;

//         if (fabs(delta) < EPSILON) {
//             std::cout << "Delta too small, stopping iteration at n = " << n << std::endl;
//             break;
//         }

//         if (fabs(x_right - x_left) < EPSILON_X) {
//             std::cout << "x interval too small, stopping iteration at n = " << n << std::endl;
//             break;
//         }

//         double x_0 = x_left - y_left * (x_right - x_left) / delta;
//         double y_0 = f(x_0)(i);

//         if (std::isinf(y_0)) {
//             return false;
//         }

//         if (fabs(y_0) < best_y) {
//             best_x = x_0;
//             best_y = fabs(y_0);
//         }

//         if ((y_left < 0.0) == (y_0 < 0.0)) { // Same signs
//             x_left = x_0;
//             y_left = y_0;
//         }
//         else {
//             x_right = x_0;
//             y_right = y_0;
//         }
//     }

//     result = best_x;
    
//     // For debugging: print the best x and its function value
//     std::cout << "Best x: " << best_x << ", f(best x) = " << f(best_x)(i) << std::endl << std::endl;

//     return best_y < EPSILON_TERMINAL;
// }

// Brent's method for finding a zero of f in [left, right]
template <int N>
bool find_zero(std::function<Eigen::Matrix<double, N, 1>(double)> f,
                     double left, double right, unsigned i, double &result)
{
    const double tol_f       = 1e-12; // strict success on |f|
    const double tol_f_term  = 1e-6;  // relaxed success if we bail for other reasons
    const double tol_x       = 1e-14; // x tolerance used only with a small-|f| gate
    const int    max_evals   = 50;

    double a = left, b = right;
    double fa = f(a)(i);
    double fb = f(b)(i);
    int evals = 2;

    if (!std::isfinite(fa) || !std::isfinite(fb)) return false;
    if (fa == 0.0) { result = a; return true; }
    if (fb == 0.0) { result = b; return true; }
    if (fa * fb > 0.0) return false; // need a sign change to bracket

    // Ensure |f(b)| <= |f(a)| so b is the better endpoint
    if (std::fabs(fa) < std::fabs(fb)) {
        std::swap(a, b); std::swap(fa, fb);
    }

    // c is the previous best opposite-sign endpoint to b. d is the one before c.
    double c = a, fc = fa;
    double d = c;

    // Track the best-so-far point by function value
    double x_best = (std::fabs(fa) <= std::fabs(fb)) ? a : b;
    double f_best = (std::fabs(fa) <= std::fabs(fb)) ? fa : fb;

    bool mflag = true;

    while (evals < max_evals) {
        // Inverse quadratic interpolation if possible, else secant
        double s;
        if (fa != fc && fb != fc && fa != fb) {
            s = (a*fb*fc)/((fa-fb)*(fa-fc))
              + (b*fa*fc)/((fb-fa)*(fb-fc))
              + (c*fa*fb)/((fc-fa)*(fc-fb));
        } else {
            s = b - fb*(b - a)/(fb - fa);
        }

        // Accept tests, else bisect
        double s_mid = 0.5*(a + b);
        bool cond1 = (s < std::min(a, b) || s > std::max(a, b));
        bool cond2 = (mflag && std::fabs(s - b) >= 0.5*std::fabs(b - c));
        bool cond3 = (!mflag && std::fabs(s - b) >= 0.5*std::fabs(c - d));
        bool cond4 = (mflag && std::fabs(b - c) < tol_x);
        bool cond5 = (!mflag && std::fabs(c - d) < tol_x);

        if (cond1 || cond2 || cond3 || cond4 || cond5) {
            s = s_mid; // bisection
            mflag = true;
        } else {
            mflag = false;
        }

        double fs = f(s)(i);
        ++evals;
        if (!std::isfinite(fs)) return false;

        // Update best-so-far
        if (std::fabs(fs) < std::fabs(f_best)) { x_best = s; f_best = fs; }

        // Success by function value only
        if (std::fabs(fs) <= tol_f) { result = s; return true; }

        // Do not allow small interval to claim success unless |f| is also small
        if (std::fabs(b - a) <= tol_x && std::fabs(f_best) <= tol_f_term) {
            result = x_best; 
            return true;
        }

        // Shift history and maintain the sign change in [a, b]
        d = c;  c = b;  fc = fb;

        if ((fa > 0 && fs < 0) || (fa < 0 && fs > 0)) {
            b = s; fb = fs;
        } else {
            a = s; fa = fs;
        }

        // Keep b as the better endpoint
        if (std::fabs(fa) < std::fabs(fb)) {
            std::swap(a, b); std::swap(fa, fb);
        }
    }

    // Out of evaluations. Return the best point seen and report success only if |f| is small.
    result = x_best;
    return std::fabs(f_best) <= tol_f_term;
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
            // std::cout << "Found triangle pointing up at [x0, x1, x2] = [" << x[0] << ", " << x[1] << ", " << x[2] << "] with [y0, y1, y2] = [" << y[0] << ", " << y[1] << ", " << y[2] << "]" << std::endl;
            double max_x = find_max<N>(f, x[0], x[2], i);
            // std::cout << "Max found at x = " << max_x << " with f(max) = " << f(max_x)(i) << std::endl;
            if (f(max_x)(i) > 0.0) {
                double z1, z2;
                if (max_x < x[1]) { // max falls between x[0] and x[1]
                    // std::cout << "Max falls between x0 and x1" << std::endl;
                    if (find_zero(f, x[0], max_x, i, z1)) {
                        zeros.push_back(std::make_pair(z1, i));
                    }
                    if (find_zero(f, max_x, x[1], i, z2)) {
                        zeros.push_back(std::make_pair(z2, i));
                    }
                } else { // max falls between x[1] and x[2]
                    // std::cout << "Max falls between x1 and x2" << std::endl;
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
