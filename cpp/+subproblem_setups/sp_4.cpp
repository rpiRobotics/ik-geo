//---------------------------------------------------------------//
// Name: sp_4.cpp
// Author: Amar Maksumic
// Date: 11/15/2022
// Purpose: Port of the subproblem/sp_4.m file
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <iostream>
#include "sp_4.h"

using namespace Eigen;

void sp4_setup(Eigen::Vector3d& p, Eigen::Vector3d& k, 
               Eigen::Vector3d& h, double& d, Eigen::Vector2d& theta){
  p = rand_vec();
  k = rand_normal_vec();
  h = rand_normal_vec();
  theta << rand_angle(), 0;

  d = h.transpose() * rot(k, theta(0, 0)) * p;
}

void sp4_setup_LS(Eigen::Vector3d& p, Eigen::Vector3d& k, 
                  Eigen::Vector3d& h, double& d, Eigen::Vector2d& theta){
  p = rand_vec();
  k = rand_normal_vec();
  h = rand_normal_vec();
  theta << rand_angle(), 0;

  d = ((double)rand() / (double)(RAND_MAX));
}

bool sp4_run(Eigen::Vector3d& p, Eigen::Vector3d& k, 
             Eigen::Vector3d& h, double& d, Eigen::Vector2d& theta){
  Eigen::Matrix<double, 3, 1> A_11 = k.cross(p);
  Eigen::Matrix<double, 3, 2> A_1;
  A_1 << A_11, -k.cross(A_11);

  Eigen::Matrix<double, 1, 2> A = h.transpose() * A_1;

  double b = d - (h.transpose() * (k * (k.transpose() * p)));

  double norm_A_2 = A.dot(A);

  Eigen::Matrix<double, 2, 1> x_ls = A_1.transpose() * (h * b);

  if (norm_A_2 > b*b) {
    double sqrt_2_b = norm_A_2 - b * b;
    double xi = sqrt(sqrt_2_b);

    Eigen::Matrix<double, 2, 1> A_perp_tilde;
    A_perp_tilde << A(1), -A(0);

    Eigen::Matrix<double, 2, 1> sc_1 = x_ls + xi*A_perp_tilde;
    Eigen::Matrix<double, 2, 1> sc_2 = x_ls - xi*A_perp_tilde;

    double theta_1 = atan2(sc_1(0, 0), sc_1(1, 0));
    double theta_2 = atan2(sc_2(0, 0), sc_2(1, 0));
    theta << theta_1,  theta_2;
    return false;
  } else {
    theta << atan2(x_ls(0, 0), x_ls(1, 0)), 0;
    return true;
  }
}

double sp4_error(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                 Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
                 double& theta1, double& theta2){
   return (rot(k2, theta2) * p2 - rot(k1, theta1) * p1).norm();
}