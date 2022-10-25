#include <iostream>
#include <eigen3/Eigen/Dense>
#include "../rand_cpp.h"

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

void sp_2E(Vector3d &p0, Vector3d &p1, Vector3d &p2, Vector3d &k1, Vector3d &k2, 
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

	Vector3d p0, p1, p2, k1, k2;
	double theta1, theta2;

	int num = 0;

	for (int arg = 1; arg < argc; arg ++ ) {
		if (argv[arg] == std::string("-n")) {
			num = atoi(argv[ ++ arg]);
		}
	}

	clock_t begin = clock();

	for (int i = 0; i < num; i ++ ) {
		sp_2E_setup(p0, p1, p2, k1, k2, theta1, theta2);
		sp_2E(p0, p1, p2, k1, k2, theta1, theta2);
	}

	clock_t end = clock();

	double timeSec = (end - begin) / static_cast<double>( CLOCKS_PER_SEC );

	std::cout << "===== \n time (seconds): " << timeSec << std::endl;

	return 0;
}