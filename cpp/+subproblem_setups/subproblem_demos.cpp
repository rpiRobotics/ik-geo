//---------------------------------------------------------------//
// Name: subproblem_demo.cpp
// Author: Amar Maksumic
// Date: 04/17/2022
// Purpose: Demos of how to use subproblems 1 through 6
//---------------------------------------------------------------//

#include "sp_1.cpp"
#include "sp_2.cpp"
#include "sp_2E.cpp"
#include "sp_3.cpp"
#include "sp_4.cpp"
#include "sp_5.cpp"
#include "sp_6.cpp"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

std::string to_string(std::vector<double> vec) {
  std::string str = "";
  for (unsigned int i = 0; i < vec.size(); i++) {
    str += std::to_string(vec[i]) + " ";
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
  is_LS = sp1_run(p1, p2, k, theta);
  std::cout << "Subproblem 1" << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "k: " << k << std::endl;
  std::cout << "theta: " << theta << std::endl;
  std::cout << "is_LS: " << is_LS << std::endl;
}

// This is a demo of how to use subproblem 2
// Input: 3x1 vector p1, 3x1 vector p2, 3x1 vector k1, 3x1 vector k2
// Output: std::vectors of doubles theta1 and theta2 (through reference parameter) and bool is_LS (return value)
void sp2_demo() {
  Eigen::Vector3d p1, p2, k1, k2;
  std::vector<double> theta1, theta2;
  bool is_LS;
  sp2_setup(p1, p2, k1, k2, theta1, theta2);
  is_LS = sp2_run(p1, p2, k1, k2, theta1, theta2);
  std::cout << "Subproblem 2" << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "k1: " << k1 << std::endl;
  std::cout << "k2: " << k2 << std::endl;
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
  sp_2E_setup(p0, p1, p2, k1, k2, theta1, theta2);
  sp_2E(p0, p1, p2, k1, k2, theta1, theta2);
  std::cout << "Subproblem 2E" << std::endl;
  std::cout << "p0: " << p0 << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "k1: " << k1 << std::endl;
  std::cout << "k2: " << k2 << std::endl;
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
  sp_3_setup(p1, p2, k, d);
  is_LS = sp_3(p1, p2, k, d, theta);
  std::cout << "Subproblem 3" << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "k: " << k << std::endl;
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
  is_LS = sp4_run(p, k, h, d, theta);
  std::cout << "Subproblem 4" << std::endl;
  std::cout << "p: " << p << std::endl;
  std::cout << "k: " << k << std::endl;
  std::cout << "h: " << h << std::endl;
  std::cout << "d: " << d << std::endl;
  std::cout << "theta: " << to_string(theta) << std::endl;
  std::cout << "is_LS: " << is_LS << std::endl;
}

// This is a demo of how to use subproblem 5
// Input: 3x1 vector p0, 3x1 vector p1, 3x1 vector p2, 3x1 vector p3,
//        3x1 vector k1, 3x1 vector k2, 3x1 vector k3
// Output: std::vectors theta1, theta2, theta3 (through reference parameter)
void sp5_demo() {
  Eigen::Vector3d p0, p1, p2, p3, k1, k2, k3;
  std::vector<double> theta1, theta2, theta3;
  sp_5_setup(p0, p1, p2, p3, k1, k2, k3);
  sp_5(p0, p1, p2, p3, k1, k2, k3, theta1, theta2, theta3);
  std::cout << "Subproblem 5" << std::endl;
  std::cout << "p0: " << p0 << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "p3: " << p3 << std::endl;
  std::cout << "k1: " << k1 << std::endl;
  std::cout << "k2: " << k2 << std::endl;
  std::cout << "k3: " << k3 << std::endl;
  std::cout << "theta1: " << to_string(theta1) << std::endl;
  std::cout << "theta2: " << to_string(theta2) << std::endl;
  std::cout << "theta3: " << to_string(theta3) << std::endl;
}

// This is a demo of how to use subproblem 6
// Input: 3x4 vector h, 3x4 vector k, 3x4 vector p, double d1, double d2
// Output: 4 long std::vectors theta1 and theta2 (through reference parameter)
void sp6_demo() {
  Eigen::Matrix<double, 3, 4> h, k, p;
  double d1, d2;
  std::vector<double> theta1, theta2;
  sp6_setup(h, k, p, d1, d2, theta1, theta2);
  sp6_run(h, k, p, d1, d2, theta1, theta2);
  std::cout << "Subproblem 6" << std::endl;
  std::cout << "h: " << h << std::endl;
  std::cout << "k: " << k << std::endl;
  std::cout << "p: " << p << std::endl;
  std::cout << "d1: " << d1 << std::endl;
  std::cout << "d2: " << d2 << std::endl;
  std::cout << "theta1: " << to_string(theta1) << std::endl;
  std::cout << "theta2: " << to_string(theta2) << std::endl;
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