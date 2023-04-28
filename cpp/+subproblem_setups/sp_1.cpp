//---------------------------------------------------------------//
// Name: sp_1.cpp
// Author: Ashton Ropp, Runbin Chen, Amar Maksumic
// Date: 10/15/2022
// Purpose: Port of the subproblem/sp_1.m file
//---------------------------------------------------------------//
#pragma GCC optimize(3)

#include <iostream>
#include <chrono>
#include "sp_1.h"
#include "../read_csv.h"

void sp1_setup(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& k, double& theta){
  p1 = rand_vec();
  k = rand_normal_vec();
  theta = rand_angle();

  p2 = rot(k, theta) * p1;
}

// return is_LS
bool sp1_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
             const Eigen::Vector3d& k, double& theta){

	Eigen::Matrix<double, 3, 1> KxP = k.cross(p1);
	Eigen::Matrix<double, 3, 2> A;
	A << KxP, -k.cross(KxP);

	Eigen::Vector2d x = A.transpose() * p2;

	theta = atan2(x(0), x(1));

	return fabs(p1.norm() - p2.norm()) > ZERO_THRESH || fabs(k.dot(p1) - k.dot(p2)) > ZERO_THRESH;
   
}


void sp1_setup_LS(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                  Eigen::Vector3d& k, double& theta){
  p1 = rand_vec();
  k = rand_normal_vec();
  theta = rand_angle();

  p2 = rand_vec();
}

double sp1_error(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
                 const Eigen::Vector3d& k, double& theta){
  return (p2 - rot(k, theta)*p1).norm();
}