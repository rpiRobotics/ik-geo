//---------------------------------------------------------------//
// Name: sp_2.h
// Author: Amar Maksumic
// Date: 11/01/2022
// Purpose: Port of the subproblem/sp_2.m file
//---------------------------------------------------------------//

#include <iostream>
#include "sp_2.h"

void sp2_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
               Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
               double& theta1, double& theta2){
  p1 = rand_vec();
  k1 = rand_normal_vec();
  k2 = rand_normal_vec();
  theta1 = rand_angle();
  theta2 = rand_angle();

  p2 = rot(k2, theta2) * rot(k1, theta1) * p1;
}

void sp2_setup_LS(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                  Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
                  double& theta1, double& theta2){
  p1 = rand_vec();
  k1 = rand_normal_vec();
  k2 = rand_normal_vec();
  theta1 = rand_angle();
  theta2 = rand_angle();

  p2 = rand_vec();
}

bool sp2_run(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
             Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
             double& theta1, double& theta2){

  p1 = p1/p1.norm();
  p2 = p2/p2.norm();

  Eigen::Matrix<double, 3, 1> KxP1 = k1.cross(p1);
  Eigen::Matrix<double, 3, 1> KxP2 = k2.cross(p2);

  Eigen::Matrix<double, 3, 2> A_1, A_2; 
  A_1 << KxP1, -k1.cross(KxP1);
  A_2 << KxP2, -k2.cross(KxP2);

  double radius_1_sq = KxP1.dot(KxP1);
  double radius_2_sq = KxP2.dot(KxP2);

  double k1_d_p1 = k1.dot(p1);
  double k2_d_p2 = k2.dot(p2);
  double k1_d_k2 = k1.dot(k2);
  
  double ls_frac = 1/(1-(k1_d_k2*k1_d_k2));
  double alpha_1 = ls_frac * (k1_d_p1 - k1_d_k2 * k2_d_p2);
  double alpha_2 = ls_frac * (k2_d_p2 - k1_d_k2 * k1_d_p1);

  Eigen::Matrix<double, 2, 1> x_ls_1 = (alpha_2 * A_1.transpose() * (k2)) / radius_1_sq;
  Eigen::Matrix<double, 2, 1> x_ls_2 = (alpha_1 * A_2.transpose() * (k1)) / radius_2_sq;
  Eigen::Matrix<double, 4, 1> x_ls;
  x_ls << x_ls_1, x_ls_2; 
  
  Eigen::Matrix<double, 3, 1> n_sym = k1.cross(k2);
  Eigen::Matrix<double, 2, 3> pinv_A1, pinv_A2;
  pinv_A1 = A_1.transpose() / radius_1_sq;
  pinv_A2 = A_2.transpose() / radius_2_sq;

	Eigen::Matrix<double, 4, 1> A_perp_tilde;
	Eigen::Matrix<double, 4, 3> temp;
	temp << pinv_A1, pinv_A2;
	A_perp_tilde = temp * n_sym;

  if (x_ls.block<2, 1>(0,0).norm() < 1) {
    double xi = sqrt(1 - pow(x_ls.block<2, 1>(0,0).norm(), 2)) / A_perp_tilde.block<2, 1>(0, 0).norm();
    Eigen::Matrix<double, 4, 1> sc_1 = x_ls + xi*A_perp_tilde;
    Eigen::Matrix<double, 4, 1> sc_2 = x_ls - xi*A_perp_tilde;

    theta1 = atan2(sc_1(0, 0), sc_1(1, 0)) + atan2(sc_2(0, 0), sc_2(1, 0));
    theta2 = atan2(sc_1(0, 0), sc_1(1, 0)) + atan2(sc_2(0, 0), sc_2(1, 0));
    return false;
  } else {
    theta1 = atan2(x_ls(0, 0), x_ls(1, 0));
    theta2 = atan2(x_ls(2, 0), x_ls(3, 0));
    return true;
  }
}

double sp2_error(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                 Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
                 double& theta1, double& theta2){
  return (rot(k2, theta2) * p2 - rot(k1, theta1) * p1).norm();
}