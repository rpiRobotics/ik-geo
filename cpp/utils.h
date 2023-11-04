#ifndef UTILS_H_
#define UTILS_H_

#define _USE_MATH_DEFINES
#include <cmath>

#include <time.h>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

struct Kinematics {
    Eigen::Matrix<double, 3, 6> H;
    Eigen::Matrix<double, 3, 7> P;

    void forward_kinematics(const Eigen::Matrix<double, 6, 1> &theta, Eigen::Matrix<double, 3, 3> &r, Eigen::Matrix<double, 3, 1> &p);
};

struct Solution {
	std::vector<Eigen::Matrix<double, 6, 1>> q;
	std::vector<bool> is_ls;
};

//Create number from 0.0000 to 1.0000
double rand_0to1();

//Example input is a normal vector
//Matrix cross-product for 3 x 3 vector
Eigen::Matrix<double, 3, 3> hat(const Eigen::Matrix<double, 3, Eigen::Dynamic>& vec);

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
Eigen::Matrix<double, 3, Eigen::Dynamic> rot(Eigen::Matrix<double, 3, Eigen::Dynamic> k, double theta);

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

#endif // UTILS_H_
