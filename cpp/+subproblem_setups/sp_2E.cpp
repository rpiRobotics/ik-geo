//---------------------------------------------------------------//
// Name: sp_2E.cpp
// Author: Runbin Chen
// Date: 10/25/2022
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <chrono>
#include "sp_2E.h"
#include "../read_csv.h"

using namespace Eigen;

void sp_2E_setup(Vector3d &p0, Vector3d &p1, Vector3d &p2, Vector3d &k1, Vector3d &k2, 
		  double &theta1, double &theta2) {
	p0 = rand_vec();
	p1 = rand_vec();
	p2 = rand_vec();
	k1 = rand_normal_vec();
	k2 = rand_normal_vec();
	theta1 = rand_angle();
	theta2 = rand_angle();
}

void sp_2E(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2, const Vector3d &k1, const Vector3d &k2, 
		  double &theta1, double &theta2) {
	Matrix<double, 3, 1> KxP1 = k1.cross(p1);
	Matrix<double, 3, 1> KxP2 = k2.cross(p2);

	Matrix<double, 3, 2> A_1, A_2;
	A_1 << KxP1, -k1.cross(KxP1);
	A_2 << KxP2, -k2.cross(KxP2);

	Matrix<double, 3, 4> A;
	A << A_1, -A_2;

	Vector3d p = -k1*k1.dot(p1) + k2*k2.dot(p2) - p0;

	double radius_1_sp = KxP1.dot(KxP1);
	double radius_2_sp = KxP2.dot(KxP2);

	double alpha = radius_1_sp / (radius_1_sp + radius_2_sp);
	double beta = radius_2_sp / (radius_1_sp + radius_2_sp);
	Matrix<double, 3, 3> M_inv, AAT_inv;
	M_inv = Matrix3d::Identity(3, 3) + k1*k1.transpose()*(alpha/(1-alpha));
	AAT_inv = 1/(radius_1_sp+radius_2_sp)*(M_inv + M_inv*k2*k2.transpose()*M_inv*beta/(1.0-(k2.transpose()*M_inv*k2*beta)(0, 0)));
	Matrix<double, 4, 1> x_ls = A.transpose() * AAT_inv * p;

	Matrix<double, 3, 1> n_sym = k1.cross(k2);
	Matrix<double, 2, 3> pinv_A1, pinv_A2;
	pinv_A1 = A_1.transpose() / radius_1_sp;
	pinv_A2 = A_2.transpose() / radius_2_sp;
	Matrix<double, 4, 1> A_perp_tilde;
	Matrix<double, 4, 3> temp;
	temp << pinv_A1, 
			pinv_A2;
	A_perp_tilde = temp * n_sym;

	double num = (pow(x_ls.block<2, 1>(2, 0).norm(), 2)-1)*pow(A_perp_tilde.block<2, 1>(0, 0).norm(), 2) 
				- (pow(x_ls.block<2, 1>(0, 0).norm(), 2)-1)*pow(A_perp_tilde.block<2, 1>(2, 0).norm(), 2);
	double den = 2*(x_ls.block<2, 1>(0, 0).transpose()*A_perp_tilde.block<2, 1>(0, 0)*pow(A_perp_tilde.block<2, 1>(2, 0).norm(), 2) \
				- x_ls.block<2, 1>(2, 0).transpose()*A_perp_tilde.block<2, 1>(2, 0)*pow(A_perp_tilde.block<2, 1>(0, 0).norm(), 2))(0, 0);

	double xi = num / den;

	Matrix<double, 4, 1> sc = x_ls + xi*A_perp_tilde;

	theta1 = atan2(sc(0, 0), sc(1, 0));
	theta2 = atan2(sc(2, 0), sc(3, 0));
}

int main(int argc, char* argv[]) {
	std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("sp_2E.csv");
  	if (data.size() != 17) {
    	std::cerr << "Invalid input data. \n";
    	return 0;
  	}

  	double time_avg = 0;

  	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
	  	Eigen::Vector3d p0, p1, p2, k1, k2;
	   	double theta1, theta2;
		p0 << data[0].second[i], data[1].second[i], data[2].second[i];
		p1 << data[3].second[i], data[4].second[i], data[5].second[i];
		p2 << data[12].second[i], data[13].second[i], data[14].second[i];
		k1 << data[6].second[i], data[7].second[i], data[8].second[i];
	    k2 << data[9].second[i], data[10].second[i], data[11].second[i];
	    theta1 = data[15].second[i];
	    theta2 = data[16].second[i];

	    auto start = std::chrono::steady_clock::now();

	    sp_2E(p0, p1, p2, k1, k2, theta1, theta2);

	    auto end = std::chrono::steady_clock::now();

	    time_avg += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }

  	time_avg /= (int)data[0].second.size();

  	std::cout << "===== \n time (microseconds): " << time_avg << std::endl;

	return 0;
}