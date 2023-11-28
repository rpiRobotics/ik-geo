#ifndef UTILS_H_
#define UTILS_H_

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <time.h>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

template <unsigned N1, unsigned N2>
struct Kinematics {
    Eigen::Matrix<double, 3, N1> H;
    Eigen::Matrix<double, 3, N2> P;

    void forward_kinematics(const Eigen::Matrix<double, N1, 1> &theta, Eigen::Matrix<double, 3, 3> &r, Eigen::Matrix<double, 3, 1> &p);
    std::vector<Eigen::Vector3d> forward_kinematics_inter(const Eigen::Matrix<double, N1, 1> &theta, const std::vector<unsigned> &inter,
                                                          Eigen::Matrix<double, 3, 3> &r, Eigen::Matrix<double, 3, 1> &p);
};

template <unsigned N>
struct Solution {
    std::vector<Eigen::Matrix<double, N, 1>> q;
    std::vector<bool> is_ls;
};

struct SEWConv {
    Eigen::Vector3d e_r;

    SEWConv(const Eigen::Vector3d &e_r) : e_r(e_r) {}

    double fwd_kin(const Eigen::Vector3d &S, const Eigen::Vector3d &E, const Eigen::Vector3d &W) const;
    Eigen::Vector3d inv_kin(const Eigen::Vector3d &S, const Eigen::Vector3d &W, double psi, Eigen::Vector3d &n_SEW) const;
};

//Create number from 0.0000 to 1.0000
double rand_0to1();

//Example input is a normal vector
//Matrix cross-product for 3 x 3 vector
Eigen::Matrix3d hat(const Eigen::Vector3d& vec);

//Create a random angle
double rand_angle();

Eigen::Matrix<double, Eigen::Dynamic, 1> rand_angle(int N);

//Create random matrix of width = size, height = 3
Eigen::Matrix<double, 3, Eigen::Dynamic> rand_vec(int N = 1);

Eigen::Matrix<double, 3, Eigen::Dynamic> rand_normal_vec(int size = 1);

// Only works with input vectors of size 1.  Same as MATLAB implementation
Eigen::Vector3d rand_perp_normal_vec(const Eigen::Vector3d& vec);

//Create 3x3 rotation matrix using the Euler Rodrigues formula
//TODO: Compare notes
Eigen::Matrix3d rot(const Eigen::Vector3d &k, double theta);

double wrap_to_pi(double theta);

Eigen::Vector2d solve_lower_triangular_2x2(const Eigen::Matrix2d &l, const Eigen::Vector2d &bv);

template <int N>
bool find_zero(std::function<Eigen::Matrix<double, N, 1>(double)> f, double left, double right, unsigned i, double &result) {
    const unsigned ITERATIONS = 100;
    const double EPSILON = 1e-5;

    double x_left = left;
    double x_right = right;

    double y_left = f(x_left)(i);
    double y_right = f(x_right)(i);

    for (unsigned n = 0; n < ITERATIONS; ++n) {
        double delta = y_right - y_left;

        if (fabs(delta) < EPSILON) {
            break;
        }

        double x_0 = x_left - y_left * (x_right - x_left) / delta;
        double y_0 = f(x_0)(i);

        if (std::isinf(y_0)) {
            return false;
        }

        if ((y_left < 0.0) != (y_0 < 0.0)) {
            x_left = x_0;
            y_left = y_0;
        }
        else {
            x_right = x_0;
            y_right = y_0;
        }
    }

    if (left <= x_left && x_left <= right) {
        result = x_left;
        return true;
    }
    else {
        return false;
    }
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

template <unsigned N1, unsigned N2>
void Kinematics<N1, N2>::forward_kinematics(const Eigen::Matrix<double, N1, 1> &theta, Eigen::Matrix<double, 3, 3> &r, Eigen::Matrix<double, 3, 1> &p) {
    p = P.col(0);
    r = Eigen::Matrix<double, 3, 3>::Identity();

    for (unsigned i = 0; i < N1; ++i) {
        r = r * rot(H.col(i), theta(i));
        p = p + r * P.col(i + 1);
    }
}

template <unsigned N1, unsigned N2>
std::vector<Eigen::Vector3d> Kinematics<N1, N2>::forward_kinematics_inter(const Eigen::Matrix<double, N1, 1> &theta, const std::vector<unsigned> &inter,
                                                                          Eigen::Matrix<double, 3, 3> &r, Eigen::Matrix<double, 3, 1> &p) {
    std::vector<Eigen::Vector3d> p_inter;

    p = P.col(0);
    r = Eigen::Matrix<double, 3, 3>::Identity();

    for (unsigned i = 0; i < N1; ++i) {
        if (std::find(inter.begin(), inter.end(), i) != inter.end()) {
            p_inter.push_back(p);
        }

        r = r * rot(H.col(i), theta(i));
        p = p + r * P.col(i + 1);
    }

    return p_inter;
}

#endif // UTILS_H_
