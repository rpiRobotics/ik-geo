//---------------------------------------------------------------//
// Name: sp_6.cpp
// Author: Amar Maksumic
// Date: 11/28/2022
// Purpose: Port of the subproblem/sp_6.m file
//---------------------------------------------------------------//

#include <chrono>
#include <complex>
#include <iostream>
#include "sp_6.h"
#include "../read_csv.h"
#include "../helper.h"

void sp6_setup(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
               Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2, 
               std::vector<double>& theta1, std::vector<double>& theta2) {
  //
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

void sp6_run(Eigen::Matrix<double, 3, 4>& h, Eigen::Matrix<double, 3, 4>& k, 
             Eigen::Matrix<double, 3, 4>& p, double& d1, double& d2,
             std::vector<double> &theta1, std::vector<double> &theta2) {
  Eigen::Vector3d k1Xp1 = k.col(0).cross(p.col(0));
  Eigen::Vector3d k2Xp2 = k.col(1).cross(p.col(1));
  Eigen::Vector3d k3Xp3 = k.col(2).cross(p.col(2));
  Eigen::Vector3d k4Xp4 = k.col(3).cross(p.col(3));

  Eigen::Matrix<double, 3, 2> A_1;
  A_1 << k1Xp1, -k.col(0).cross(k1Xp1);
  Eigen::Matrix<double, 3, 2> A_2;
  A_2 << k2Xp2, -k.col(1).cross(k2Xp2);
  Eigen::Matrix<double, 3, 2> A_3;
  A_3 << k3Xp3, -k.col(2).cross(k3Xp3);
  Eigen::Matrix<double, 3, 2> A_4;
  A_4 << k4Xp4, -k.col(3).cross(k4Xp4);

  Eigen::Matrix<double, 2, 4> A;
  A << (h.col(0).transpose() * A_1), (h.col(1).transpose() * A_2),
       (h.col(2).transpose() * A_3), (h.col(3).transpose() * A_4);

  Eigen::Matrix<double, 4, 1> x_min;
  Eigen::Matrix<double, 2, 1> den;
  den << (d1 - h.col(0).transpose() * k.col(0) * k.col(0).transpose() * p.col(0) - h.col(1).transpose() * k.col(1) * k.col(1).transpose() * p.col(1)),
         (d2 - h.col(2).transpose() * k.col(2) * k.col(2).transpose() * p.col(2) - h.col(3).transpose() * k.col(3) * k.col(3).transpose() * p.col(3));
  x_min = A.colPivHouseholderQr().solve(den);

  Eigen::CompleteOrthogonalDecomposition<
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> cod;
  cod.compute(A);
  unsigned rk = cod.rank();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P =
      cod.colsPermutation();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V =
      cod.matrixZ().transpose();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x_null =
      P * V.block(0, rk, V.rows(), V.cols() - rk);

  Eigen::Matrix<double, 4, 1> x_null_1 = x_null.col(0);
  Eigen::Matrix<double, 4, 1> x_null_2 = x_null.col(1);

  Eigen::Matrix<double, 4, 1> xi_1;
  Eigen::Matrix<double, 4, 1> xi_2;

  Eigen::Matrix<double, 2, 1> x_min_1 = x_min.block<2, 1>(0,0);
  Eigen::Matrix<double, 2, 1> x_min_2 = x_min.block<2, 1>(2,0);
  // std::cout << x_null.rows() << " " << x_null.cols() << std::endl;
  Eigen::Matrix<double, 2, 2> x_n_1 = x_null.block<2, 2>(0,0);
  Eigen::Matrix<double, 2, 2> x_n_2 = x_null.block<2, 2>(2,0);

  solve_2_ellipse_numeric(x_min_1, x_n_1, x_min_2, x_n_2, xi_1, xi_2);

  theta1.clear();
  theta2.clear();

  for (int i = 0; i < 4; i++) {
    Eigen::Matrix<double, 4, 1> x = x_min + x_null_1 * xi_1(i, 0) + x_null_2 * xi_2(i, 0);
    theta1.push_back(atan2(x(0, 0), x(1, 0)));
    theta2.push_back(atan2(x(2, 0), x(3, 0)));
  }
};