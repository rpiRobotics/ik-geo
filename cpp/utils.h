#ifndef UTILS_H_
#define UTILS_H_

#define _USE_MATH_DEFINES
#include <cmath>

#include <tuple>
#include <time.h>
#include <random>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <limits>
#include <optional>
#include <iostream>

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

    SEWConv() : e_r(Eigen::Vector3d::Zero()) {}
    SEWConv(const Eigen::Vector3d &e_r_arg) : e_r(e_r_arg) {} // e_r_arg to prevent shadow variable

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

// Function to wrap each element of a vector to the range [-π, π]
Eigen::VectorXd wrap_to_pi(const Eigen::VectorXd &theta);

int modulo(int a, int b);

Eigen::Vector2d solve_lower_triangular_2x2(const Eigen::Matrix2d &l, const Eigen::Vector2d &bv);

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
