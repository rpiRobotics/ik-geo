//---------------------------------------------------------------//
// Name: sp_4.cpp
// Author: Amar Maksumic
// Date: 11/28/2022
// Purpose: Port of the subproblem/sp_6.m file
//---------------------------------------------------------------//

#include <iostream>
#include "sp_6.h"

using namespace Eigen;

void sp6_setup(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
               Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2, 
               double& theta1, double& theta2) {
  //
  h = rand_normal_vec(4);
  k = rand_normal_vec(4);
  p = rand_normal_vec(4);

  theta1 = rand_angle();
  theta2 = rand_angle();

  d1 = 
  d2 = 
}

bool sp6_run(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
             Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2);