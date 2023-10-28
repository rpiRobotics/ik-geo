#define _USE_MATH_DEFINES
#include <cmath>

#include "utils.h"
#include <utility>

//Create number from 0.0000 to 1.0000
double rand_0to1(){
    srand(time(NULL));
    return (double) rand() / RAND_MAX;
}

//Example input is a normal vector
//Matrix cross-product for 3 x 3 vector
Eigen::Matrix<double, 3, 3> hat(const Eigen::Matrix<double, 3, Eigen::Dynamic>& vec){
    Eigen::Matrix3d output;
    output << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return output;
}

//Create a random angle
double rand_angle(){
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(0, 1);

    double theta = distr(eng)*2*M_PI-M_PI;

    return theta;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> rand_angle(int N){
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(0, N);
    Eigen::Matrix<double, Eigen::Dynamic, 1> theta;

    for (int i = 0; i < N; i++) {
        theta(i, 0) = distr(eng)*2*M_PI-M_PI;
    }

    return theta;
}

//Create random matrix of width = size, height = 3
Eigen::Matrix<double, 3, Eigen::Dynamic> rand_vec(int N){
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(0, 1);
    Eigen::Matrix<double, 3, Eigen::Dynamic> vec(3, N);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < N; j++) {
            vec(i, j) = (distr(eng)*2-1);
        }
    }
    return vec;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> rand_normal_vec(int size){
    Eigen::Matrix<double, 3, Eigen::Dynamic> v = rand_vec(size);
    v = v / v.norm();
    return v;
}

// Only works with input vectors of size 1.  Same as MATLAB implementation
Eigen::Vector3d rand_perp_normal_vec(Eigen::Vector3d& vec){
    Eigen::Vector3d randCross = rand_vec();
    randCross = randCross.cross(vec);
    return randCross/randCross.norm();
}

//Create 3x3 rotation matrix using the Euler Rodrigues formula
//TODO: Compare notes
Eigen::Matrix<double, 3, Eigen::Dynamic> rot(Eigen::Matrix<double, 3, Eigen::Dynamic> k, double theta){
    Eigen::Matrix3d eye;
    eye = Eigen::Matrix3d::Identity();
    eye.setIdentity();
    k = k/k.norm();
    return eye + sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
}

double wrap_to_pi(double theta) {
    return fmod(theta + M_PI, M_PI * 2) - M_PI;
}

Eigen::Vector2d solve_lower_triangular_2x2(const Eigen::Matrix2d &l, const Eigen::Vector2d &bv) {
    Eigen::Vector2d result;

    double a = l(0, 0);
    double b = l(1, 0);
    double c = l(1, 1);
    double p = bv(0);
    double q = bv(1);

    double x = p / a;

    result << x, (q - x * b) / c;
    return result;
}
