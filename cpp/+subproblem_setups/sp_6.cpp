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

void find_quartic_roots(Eigen::Matrix<double, 5, 1>& coeffs, 
                        Eigen::Matrix<std::complex<double>, 4, 1>& roots) {
  /* Find the roots of a quartic polynomial */
  std::complex<double> a = coeffs.coeffRef(0,0);
  std::complex<double> b = coeffs.coeffRef(1,0);
  std::complex<double> c = coeffs.coeffRef(2,0);
  std::complex<double> d = coeffs.coeffRef(3,0);
  std::complex<double> e = coeffs.coeffRef(4,0);

  std::complex<double> p1 = 2.0*c*c*c - 9.0*b*c*d + 27.0*a*d*d + 27.0*b*b*e - 72.0*a*c*e;
  std::complex<double> q1 = c*c - 3.0*b*d + 12.0*a*e;
  std::complex<double> p2 = p1 + sqrt(-4.0*q1*q1*q1 + p1*p1);
  std::complex<double> q2 = cbrt(p2.real() / 2.0);
  std::complex<double> p3 = q1 / (3.0*a*q2) + q2 / (3.0*a);
  std::complex<double> p4 = sqrt((b*b) / (4.0*a*a) - (2.0*c) / (3.0*a) + p3);
  std::complex<double> p5 = (b*b) / (2.0*a*a) - (4.0*c) / (3.0*a) - p3;
  std::complex<double> p6 = (-(b*b*b) / (a*a*a) + (4.0*b*c) / (a*a) - (8.0*d) / a) / (4.0*p4);

  roots(0,0) = -b / (4.0*a) - p4 / 2.0 - sqrt(p5 - p6) / 2.0;
  roots(1,0) = -b / (4.0*a) - p4 / 2.0 + sqrt(p5 - p6) / 2.0;
  roots(2,0) = -b / (4.0*a) + p4 / 2.0 - sqrt(p5 + p6) / 2.0;
  roots(3,0) = -b / (4.0*a) + p4 / 2.0 + sqrt(p5 + p6) / 2.0;
}


void solve_2_ellipse_numeric(Eigen::Vector2d& xm1, Eigen::Matrix<double, 2, 2>& xn1, 
                             Eigen::Vector2d& xm2, Eigen::Matrix<double, 2, 2>& xn2,
                             Eigen::Matrix<double, 4, 1>& xi_1, Eigen::Matrix<double, 4, 1>& xi_2) {
  /* solve for intersection of 2 ellipses defined by

   xm1'*xm1 + xi'*xn1'*xn1*xi  + xm1'*xn1*xi == 1
   xm2'*xm2 + xi'*xn2'*xn2*xi  + xm2'*xn2*xi == 1
   Where xi = [xi_1; xi_2] */
  
  Eigen::Matrix<double, 2, 2> A_1 = xn1.transpose() * xn1;
  double a = A_1.coeffRef(0,0);
  double b = 2*A_1.coeffRef(1,0);
  double c = A_1.coeffRef(1, 1);
  Eigen::Matrix<double, 1, 2> B_1 = 2*xm1.transpose() * xn1;
  double d = B_1.coeffRef(0,0);
  double e = B_1.coeffRef(0,1);
  double f = xm1.transpose() * xm1 - 1;

  Eigen::Matrix<double, 2, 2> A_2 = xn2.transpose() * xn2;
  double a1 = A_2.coeffRef(0,0);
  double b1 = 2*A_2.coeffRef(1,0);
  double c1 = A_2.coeffRef(1, 1);
  Eigen::Matrix<double, 1, 2> B_2 = 2*xm2.transpose() * xn2;
  double d1 = B_2.coeffRef(0,0);
  double e1 = B_2.coeffRef(0,1);
  double fq = xm2.transpose() * xm2 - 1;

  double z0 = f*a*d1*d1+a*a*fq*fq-d*a*d1*fq+a1*a1*f*f-2*a*fq*a1*f-d*d1*a1*f+a1*d*d*fq;

  double z1 = e1*d*d*a1-fq*d1*a*b-2*a*fq*a1*e-f*a1*b1*d+2*d1*b1*a*f+2*e1*fq*a*a+d1*d1*a*e-e1*d1*a*d-2*a*e1*a1*f-f*a1*d1*b+2*f*e*a1*a1-fq*b1*a*d-e*a1*d1*d+2*fq*b*a1*d;

  double z2 = e1*e1*a*a+2*c1*fq*a*a-e*a1*d1*b+fq*a1*b*b-e*a1*b1*d-fq*b1*a*b-2*a*e1*a1*e+2*d1*b1*a*e-c1*d1*a*d-2*a*c1*a1*f+b1*b1*a*f+2*e1*b*a1*d+e*e*a1*a1-c*a1*d1*d-e1*b1*a*d+2*f*c*a1*a1-f*a1*b1*b+c1*d*d*a1+d1*d1*a*c-e1*d1*a*b-2*a*fq*a1*c;

  double z3 = -2*a*a1*c*e1+e1*a1*b*b+2*c1*b*a1*d-c*a1*b1*d+b1*b1*a*e-e1*b1*a*b-2*a*c1*a1*e-e*a1*b1*b-c1*b1*a*d+2*e1*c1*a*a+2*e*c*a1*a1-c*a1*d1*b+2*d1*b1*a*c-c1*d1*a*b;

  double z4 = a*a*c1*c1-2*a*c1*a1*c+a1*a1*c*c-b*a*b1*c1-b*b1*a1*c+b*b*a1*c1+c*a*b1*b1;

  Eigen::Matrix<double, 5, 1> z(z0, z1, z2, z3, z4);
  Eigen::Matrix<std::complex<double>, 4, 1> roots;
  find_quartic_roots(z, roots);

  for (int i = 0; i < 4; i++) {
    double y_r = roots.coeffRef(i, 0).real();
    double y_sq = y_r * y_r;
    xi_2(i, 0) = y_r;

    double x_r = -(a*fq+a*c1*y_sq-a1*c*y_sq+a*e1*y_r-a1*e*y_r-a1*f)/(a*b1*y_r+a*d1-a1*b*y_r-a1*d);
    xi_1(i,0) = x_r;
  }
}



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
             double (&theta1)[4], double (&theta2)[4]) {
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

  for (int i = 0; i < 4; i++) {
    Eigen::Matrix<double, 4, 1> x = x_min + x_null_1 * xi_1(i, 0) + x_null_2 * xi_2(i, 0);
    theta1[i] = atan2(x(0, 0), x(1, 0));
    theta2[i] = atan2(x(2, 0), x(3, 0));
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
    double theta1[4], theta2[4];
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
    theta1[0] = data[38].second[i];
    theta2[0] = data[39].second[i];

    auto start = std::chrono::steady_clock::now();

    sp6_run(h, k, p, d1, d2, theta1, theta2);

    auto end = std::chrono::steady_clock::now();

    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  }

  time_avg /= (int)data[0].second.size();

  std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl;

  return 0;
}
