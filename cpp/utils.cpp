#include "utils.h"
#include "subproblems/sp.h"

double SEWConv::fwd_kin(const Eigen::Vector3d &S, const Eigen::Vector3d &E, const Eigen::Vector3d &W) const {
    Eigen::Vector3d p_SE = E - S;
    Eigen::Vector3d e_SW = (W - S).normalized();

    double psi;
    IKS::sp1_run(e_r, p_SE, e_SW, psi);
    return psi;
}

Eigen::Vector3d SEWConv::inv_kin(const Eigen::Vector3d &S, const Eigen::Vector3d &W, double psi, Eigen::Vector3d &n_SEW) const {
    Eigen::Vector3d e_SW = (W - S).normalized();
    Eigen::Vector3d e_y = e_SW.cross(e_r).normalized();

    n_SEW = rot(e_SW, psi) * e_y;
    return n_SEW.cross(e_SW);
}

//Create number from 0.0000 to 1.0000
double rand_0to1(){
    srand(time(NULL));
    return (double) rand() / RAND_MAX;
}

//Example input is a normal vector
//Matrix cross-product for 3 x 3 vector
Eigen::Matrix3d hat(const Eigen::Vector3d& vec){
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

Eigen::VectorXd rand_angle(int N) {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(0, 1);
    Eigen::VectorXd theta(N);

    for (int i = 0; i < N; i++) {
        theta(i) = distr(eng) * 2 * M_PI - M_PI;
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
Eigen::Matrix3d rot(const Eigen::Vector3d &k, double theta){
    Eigen::Matrix3d eye;
    eye = Eigen::Matrix3d::Identity();
    Eigen::Vector3d k_norm = k.normalized();
    return eye + sin(theta)*hat(k_norm)+(1-cos(theta))*hat(k_norm)*hat(k_norm);
}

double wrap_to_pi(double theta) {
    double wrapped = fmod(theta + M_PI, 2 * M_PI);
    if (wrapped < 0) {
        wrapped += 2 * M_PI;
    }
    return wrapped - M_PI;
}

// Function to wrap each element of a vector to the range [-π, π]
Eigen::Matrix<double, Eigen::Dynamic, 1> wrap_to_pi(const Eigen::Matrix<double, Eigen::Dynamic, 1>& angles) {
    Eigen::Matrix<double, Eigen::Dynamic, 1> wrapped_angles = angles;
    for (int i = 0; i < angles.size(); ++i) {
        wrapped_angles(i) = wrap_to_pi(angles(i));
    }
    return wrapped_angles;
}

int modulo(int a, int b) {
    return (a % b + b) % b;
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
