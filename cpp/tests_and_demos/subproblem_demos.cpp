//---------------------------------------------------------------//
// Name: subproblem_demo.cpp
// Author: Amar Maksumic
// Date: 04/17/2022
// Purpose: Demos of how to use subproblems 1 through 6
//---------------------------------------------------------------//

// import all subproblems
#include "../subproblems/sp.cpp"
#include "../rand_cpp.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

void sp1_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
               Eigen::Vector3d& k, 
               double& theta);
void sp2_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
               Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
               std::vector<double>& theta1, std::vector<double>& theta2);
void sp2E_setup(Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
                double &theta1, double &theta2);
void sp3_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                Eigen::Vector3d& k, 
                double& d);
void sp4_setup(Eigen::Vector3d& p, 
               Eigen::Vector3d& k, 
               Eigen::Vector3d& h, 
               double& d, 
               std::vector<double>& theta);
void sp5_setup(Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3, 
               Eigen::Vector3d& k1, Eigen::Vector3d& k2, Eigen::Vector3d& k3);
void sp6_setup(Eigen::Matrix<double, 3, 4>& p, 
               Eigen::Matrix<double, 3, 4>& k, 
               Eigen::Matrix<double, 3, 4>& h, 
               double& d1, double& d2, 
               std::vector<double>& theta1, std::vector<double>& theta2);


std::string to_string(std::vector<double> vec) {
  std::string str = "";
  for (unsigned int i = 0; i < vec.size(); i++) {
    str += std::to_string(vec[i]) + ", ";
  }
  return str;
}

// This is a demo of how to use subproblem 1
// Input: 3x1 vector p1, 3x1 vector p2, 3x1 vector k
// Output: double theta (through reference parameter) and bool is_LS (return value)
void sp1_demo() {
  Eigen::Vector3d p1, p2, k;
  double theta;
  bool is_LS;
  sp1_setup(p1, p2, k, theta);
  is_LS = IKS::sp1_run(p1, p2, k, theta);
  std::cout << "Subproblem 1" << std::endl;
  std::cout << "p1 3x1 Vector: \n" << p1 << std::endl;
  std::cout << "p2 3x1 Vector: \n" << p2 << std::endl;
  std::cout << "k 3x1 Vector: \n" << k << std::endl;
  std::cout << "theta: " << theta << std::endl;
  std::cout << "is_LS: " << is_LS << std::endl;
}

// This is a demo of how to use subproblem 2
// Input: 3x1 vector p1, 3x1 vector p2, 3x1 vector k1, 3x1 vector k2
// Output: std::vectors of doubles theta1 and theta2 (through reference parameter) and bool is_LS (return value)
//         theta1 holds all possible angles for theta1, same logic holds for theta2
//         the indexes between the two vectors are linked, so theta1[0] and theta2[0] go together
void sp2_demo() {
  Eigen::Vector3d p1, p2, k1, k2;
  std::vector<double> theta1, theta2;
  bool is_LS;
  sp2_setup(p1, p2, k1, k2, theta1, theta2);
  is_LS = IKS::sp2_run(p1, p2, k1, k2, theta1, theta2);
  std::cout << "Subproblem 2" << std::endl;
  std::cout << "p1 3x1 Vector: \n" << p1 << std::endl;
  std::cout << "p2 3x1 Vector: \n" << p2 << std::endl;
  std::cout << "k1 3x1 Vector: \n" << k1 << std::endl;
  std::cout << "k2 3x1 Vector: \n" << k2 << std::endl;
  std::cout << "theta1: " << to_string(theta1) << std::endl;
  std::cout << "theta2: " << to_string(theta2) << std::endl;
  std::cout << "is_LS: " << is_LS << std::endl;
}

// This is a demo of how to use subproblem 2E
// Input: 3x1 vector p0, 3x1 vector p1, 3x1 vector p2, 3x1 vector k1, 3x1 vector k2
// Output: doubles theta1 and theta2 (through reference parameter)
void sp2E_demo() {
  Eigen::Vector3d p0, p1, p2, k1, k2;
  double theta1, theta2;
  sp2E_setup(p0, p1, p2, k1, k2, theta1, theta2);
  IKS::sp2E_run(p0, p1, p2, k1, k2, theta1, theta2);
  std::cout << "Subproblem 2E" << std::endl;
  std::cout << "p0 3x1 Vector: \n" << p0 << std::endl;
  std::cout << "p1 3x1 Vector: \n" << p1 << std::endl;
  std::cout << "p2 3x1 Vector: \n" << p2 << std::endl;
  std::cout << "k1 3x1 Vector: \n" << k1 << std::endl;
  std::cout << "k2 3x1 Vector: \n" << k2 << std::endl;
  std::cout << "theta1: " << theta1 << std::endl;
  std::cout << "theta2: " << theta2 << std::endl;
}

// This is a demo of how to use subproblem 3
// Input: 3x1 vector p1, 3x1 vector p2, 3x1 vector k, double d
// Output: 1 long std::vector theta if is LS,
//         2 long std::vector theta if not LS (through reference parameter) 
//         and bool is_LS (return value)
void sp3_demo() {
  Eigen::Vector3d p1, p2, k;
  double d;
  std::vector<double> theta;
  bool is_LS;
  sp3_setup(p1, p2, k, d);
  is_LS = IKS::sp3_run(p1, p2, k, d, theta);
  std::cout << "Subproblem 3" << std::endl;
  std::cout << "p1 3x1 Vector: \n" << p1 << std::endl;
  std::cout << "p2 3x1 Vector: \n" << p2 << std::endl;
  std::cout << "k 3x1 Vector: \n" << k << std::endl;
  std::cout << "d: " << d << std::endl;
  std::cout << "theta: " << to_string(theta) << std::endl;
  std::cout << "is_LS: " << is_LS << std::endl;
}

// This is a demo of how to use subproblem 4
// Input: 3x1 vector p, 3x1 vector k, 3x1 vector h, double d
// Output: 1 long std::vector theta if is LS,
//         2 long std::vector theta if not LS (through reference parameter)
//         and bool is_LS (return value)
void sp4_demo() {
  Eigen::Vector3d p, k, h;
  double d;
  std::vector<double> theta;
  bool is_LS;
  sp4_setup(p, k, h, d, theta);
  is_LS = IKS::sp4_run(p, k, h, d, theta);
  std::cout << "Subproblem 4" << std::endl;
  std::cout << "p 3x1 Vector: \n" << p << std::endl;
  std::cout << "k 3x1 Vector: \n" << k << std::endl;
  std::cout << "h 3x1 Vector: \n" << h << std::endl;
  std::cout << "d: " << d << std::endl;
  std::cout << "theta: " << to_string(theta) << std::endl;
  std::cout << "is_LS: " << is_LS << std::endl;
}

// This is a demo of how to use subproblem 5
// Input: 3x1 vector p0, 3x1 vector p1, 3x1 vector p2, 3x1 vector p3,
//        3x1 vector k1, 3x1 vector k2, 3x1 vector k3
// Output: std::vectors theta1, theta2, theta3 (through reference parameter)
//         theta1 holds all possible angles for theta1, same logic holds for theta2 and theta3
//         the indexes between the two vectors are linked, so theta1[0], theta2[0], and theta3[0] go together
void sp5_demo() {
  Eigen::Vector3d p0, p1, p2, p3, k1, k2, k3;
  std::vector<double> theta1, theta2, theta3;
  sp5_setup(p0, p1, p2, p3, k1, k2, k3);
  IKS::sp5_run(p0, p1, p2, p3, k1, k2, k3, theta1, theta2, theta3);
  std::cout << "Subproblem 5" << std::endl;
  std::cout << "p0 3x1 Vector: \n" << p0 << std::endl;
  std::cout << "p1 3x1 Vector: \n" << p1 << std::endl;
  std::cout << "p2 3x1 Vector: \n" << p2 << std::endl;
  std::cout << "p3 3x1 Vector: \n" << p3 << std::endl;
  std::cout << "k1 3x1 Vector: \n" << k1 << std::endl;
  std::cout << "k2 3x1 Vector: \n" << k2 << std::endl;
  std::cout << "k3 3x1 Vector: \n" << k3 << std::endl;
  std::cout << "theta1: " << to_string(theta1) << std::endl;
  std::cout << "theta2: " << to_string(theta2) << std::endl;
  std::cout << "theta3: " << to_string(theta3) << std::endl;
}

// This is a demo of how to use subproblem 6
// Input: 3x4 vector h, 3x4 vector k, 3x4 vector p, double d1, double d2
// Output: 4 long std::vectors theta1 and theta2 (through reference parameter)
//         theta1 holds all possible angles for theta1, same logic holds for theta2
//         the indexes between the two vectors are linked, so theta1[0] and theta2[0] go together
void sp6_demo() {
  Eigen::Matrix<double, 3, 4> h, k, p;
  double d1, d2;
  std::vector<double> theta1, theta2;
  sp6_setup(p, k, h, d1, d2, theta1, theta2);
  IKS::sp6_run(p, k, h, d1, d2, theta1, theta2);
  std::cout << "Subproblem 6" << std::endl;
  std::cout << "p 3x4 Vector: \n" << p << std::endl;
  std::cout << "k 3x4 Vector: \n" << k << std::endl;
  std::cout << "h 3x4 Vector: \n" << h << std::endl;
  std::cout << "d1: " << d1 << std::endl;
  std::cout << "d2: " << d2 << std::endl;
  std::cout << "theta1: " << to_string(theta1) << std::endl;
  std::cout << "theta2: " << to_string(theta2) << std::endl;
}

void sp1_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
               Eigen::Vector3d& k, 
               double& theta) {
  p1 = rand_vec();
  k = rand_normal_vec();
  theta = rand_angle();
  p2 = rot(k, theta) * p1;
}

void sp2_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
               Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
               std::vector<double>& theta1, std::vector<double>& theta2) {
  p1 = rand_vec();
  k1 = rand_normal_vec();
  k2 = rand_normal_vec();
  theta1.push_back(rand_angle());
  theta2.push_back(rand_angle());
  p2 = rot(k2, theta2[0]) * rot(k1, theta1[0]) * p1;
}

void sp2E_setup(Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
                double &theta1, double &theta2) {
	p0 = rand_vec();
	p1 = rand_vec();
	p2 = rand_vec();
	k1 = rand_normal_vec();
	k2 = rand_normal_vec();
	theta1 = rand_angle();
	theta2 = rand_angle();
}

void sp3_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                Eigen::Vector3d& k, 
                double& d) {
	p1 = rand_vec();
	p2 = rand_vec();
	k = rand_normal_vec();
	d = rand_0to1();
}

void sp4_setup(Eigen::Vector3d& p, 
               Eigen::Vector3d& k, 
               Eigen::Vector3d& h, 
               double& d, 
               std::vector<double>& theta) {
  p = rand_vec();
  k = rand_normal_vec();
  h = rand_normal_vec();
  theta.push_back(rand_angle());
  theta.push_back(0);

  d = h.transpose() * rot(k, theta[0]) * p;
}

void sp5_setup(Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3, 
               Eigen::Vector3d& k1, Eigen::Vector3d& k2, Eigen::Vector3d& k3) {
	p0 = rand_vec();
	p1 = rand_vec();
	p2 = rand_vec();
	p3 = rand_vec();
	k1 = rand_normal_vec();
	k2 = rand_normal_vec();
	k3 = rand_normal_vec();
}

void sp6_setup(Eigen::Matrix<double, 3, 4>& p, 
               Eigen::Matrix<double, 3, 4>& k, 
               Eigen::Matrix<double, 3, 4>& h, 
               double& d1, double& d2, 
               std::vector<double>& theta1, std::vector<double>& theta2) {
  h = rand_normal_vec(4);
  k = rand_normal_vec(4);
  p = rand_normal_vec(4);

  theta1.push_back(rand_angle());
  theta2.push_back(rand_angle());
  
  double d1_a = (h.col(0).transpose() * rot(k.col(0), theta1[0]) * p.col(0));
  double d1_b = (h.col(1).transpose() * rot(k.col(1), theta2[0]) * p.col(1));
  d1 = d1_a + d1_b;

  double d2_a = (h.col(2).transpose() * rot(k.col(2), theta1[0]) * p.col(2));
  double d2_b = (h.col(3).transpose() * rot(k.col(3), theta2[0]) * p.col(3));
  d2 = d2_a + d2_b;
}

int main() {
  sp1_demo();
  std::cout << "\n===============" << std::endl;
  sp2_demo();
  std::cout << "\n===============" << std::endl;
  sp2E_demo();
  std::cout << "\n===============" << std::endl;
  sp3_demo();
  std::cout << "\n===============" << std::endl;
  sp4_demo();
  std::cout << "\n===============" << std::endl;
  sp5_demo();
  std::cout << "\n===============" << std::endl;
  sp6_demo();
  return 0;
}