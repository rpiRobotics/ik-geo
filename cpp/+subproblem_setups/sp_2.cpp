//---------------------------------------------------------------//
// Name: sp_2.h
// Author: Amar Maksumic
// Date: 11/01/2022
// Purpose: Port of the subproblem/sp_2.m file
//---------------------------------------------------------------//

#include <chrono>
#include <iostream>
#include "sp_2.h"
#include "../read_csv.h"

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

bool sp2_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
             const Eigen::Vector3d& k1, const Eigen::Vector3d& k2, 
             std::vector<double>& theta1, std::vector<double>& theta2){

  Eigen::Vector3d p_1 = p1/p1.norm();
  Eigen::Vector3d p_2 = p2/p2.norm();

  Eigen::Matrix<double, 3, 1> KxP1 = k1.cross(p_1);
  Eigen::Matrix<double, 3, 1> KxP2 = k2.cross(p_2);

  Eigen::Matrix<double, 3, 2> A_1, A_2; 
  A_1 << KxP1, -k1.cross(KxP1);
  A_2 << KxP2, -k2.cross(KxP2);

  double radius_1_sq = KxP1.dot(KxP1);
  double radius_2_sq = KxP2.dot(KxP2);

  double k1_d_p1 = k1.dot(p_1);
  double k2_d_p2 = k2.dot(p_2);
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
    
    theta1.clear();
    theta1.push_back(atan2(sc_1(0, 0), sc_1(1, 0)));
    theta1.push_back(atan2(sc_2(0, 0), sc_2(1, 0)));
    theta2.clear();
    theta2.push_back(atan2(sc_1(2, 0), sc_1(3, 0)));
    theta2.push_back(atan2(sc_2(2, 0), sc_2(3, 0)));
    return false;
  } else {
    theta1.clear();
    theta1.push_back(atan2(x_ls(0, 0), x_ls(1, 0)));
    theta2.clear();
    theta2.push_back((x_ls(2, 0), x_ls(3, 0)));
    return true;
  }
}

double sp2_error(Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
                 Eigen::Vector3d& k1, Eigen::Vector3d& k2, 
                 std::vector<double>& theta1, std::vector<double>& theta2){
  return (rot(k2, theta2[0]) * p2 - rot(k1, theta1[0]) * p1).norm();
}

int main(int argc, char* argv[]) {
  std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("sp_2.csv");
  if (data.size() != 14) {
    std::cerr << "Invalid data for sp2.\n";
    return 0;
  }

  double time_avg = 0;

  for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
    Eigen::Vector3d p1, p2, k1, k2;
    std::vector<double> theta1, theta2;
    p1 << data[0].second[i], data[1].second[i], data[2].second[i];
    k1 << data[3].second[i], data[4].second[i], data[5].second[i];
    k2 << data[6].second[i], data[7].second[i], data[8].second[i];
    p2 << data[9].second[i], data[10].second[i], data[11].second[i];
    theta1.push_back(data[12].second[i]);
    theta2.push_back(data[13].second[i]);

    auto start = std::chrono::steady_clock::now();

    sp2_run(p1, p2, k1, k2, theta1, theta2);

    auto end = std::chrono::steady_clock::now();

    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  }

  time_avg /= (int)data[0].second.size();

  std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl;

  return 0;
}
