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
               double& theta1, double& theta2) {
  //
  h = rand_normal_vec(4);
  k = rand_normal_vec(4);
  p = rand_normal_vec(4);

  theta1 = rand_angle();
  theta2 = rand_angle();
  
  double d1_a = (h.col(0).transpose() * rot(k.col(0), theta1) * p.col(0));
  double d1_b = (h.col(1).transpose() * rot(k.col(1), theta2) * p.col(1));
  d1 = d1_a + d1_b;

  double d2_a = (h.col(2).transpose() * rot(k.col(2), theta1) * p.col(2));
  double d2_b = (h.col(3).transpose() * rot(k.col(3), theta2) * p.col(3));
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


int main(int argc, char* argv[]) {
  std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("sp_6.csv");
  if (data.size() != 40) {
    std::cerr << "Invalid data for sp6.\n";
    return 0;
  }

  double time_avg = 0;

  for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
    Eigen::Matrix<double, 3, 4> h, k, p;
    double d1, d2;
    std::vector<double> theta1, theta2;
    h << data[0].second[i], data[1].second[i], data[2].second[i], data[3].second[i],
         data[4].second[i], data[5].second[i], data[6].second[i], data[7].second[i],
         data[8].second[i], data[9].second[i], data[10].second[i], data[11].second[i];
    k << data[12].second[i], data[13].second[i], data[14].second[i], data[15].second[i],
         data[16].second[i], data[17].second[i], data[18].second[i], data[19].second[i],
         data[20].second[i], data[21].second[i], data[22].second[i], data[23].second[i];
    p << data[24].second[i], data[25].second[i], data[26].second[i], data[27].second[i],
         data[28].second[i], data[29].second[i], data[30].second[i], data[31].second[i],
         data[32].second[i], data[33].second[i], data[34].second[i], data[35].second[i];
    d1 = data[36].second[i];
    d2 = data[37].second[i];
    theta1.push_back(data[38].second[i]);
    theta2.push_back(data[39].second[i]);

    auto start = std::chrono::steady_clock::now();

    sp6_run(h, k, p, d1, d2, theta1, theta2);

    auto end = std::chrono::steady_clock::now();

    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  }

  time_avg /= (int)data[0].second.size();

  std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl;

  return 0;
}
