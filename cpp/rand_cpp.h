#ifndef __rand_cpp__
#define __rand_cpp__

#include <time.h>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Core>
#include <vector>

//Create number from 0.0000 to 1.0000
double rand_0to1(){
	srand(time(NULL));
   return (rand() % 10001) / 10000;
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
Eigen::Matrix<double, 3, Eigen::Dynamic> rand_vec(int N = 1){
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

Eigen::Matrix<double, 3, Eigen::Dynamic> rand_normal_vec(int size = 1){
  Eigen::Matrix<double, 3, Eigen::Dynamic> v = rand_vec(size);
  v = v / v.norm();
  return v;
}

// Only works with input vectors of size 1.  Same as MATLAB implementation
Eigen::VectorXd rand_perp_normal_vec(const Eigen::Vector3d& vec){
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

#endif