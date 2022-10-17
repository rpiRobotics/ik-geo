//---------------------------------------------------------------//
// Name: sp_1.h
// Author: Ashton Ropp, Runbin Chen
// Date: 10/15/2022
// Purpose: Port of the subproblem/sp_1.m file
//---------------------------------------------------------------//

#include <iostream>
#include "sp_1.h"

using namespace Eigen;

// return is_LS
bool sp_1(Vector3d p1, Vector3d p2, Vector3d k, 
		  double &theta) {
	// p2 = rot(k, theta) * p1

	Matrix<double, 3, 1> KxP = k.cross(p1);
	Matrix<double, 3, 2> A;
	A << KxP, -k.cross(KxP);

	Vector2d x = A.transpose() * p2;

	theta = atan2(x(0), x(1));

	return fabs(p1.norm() - p2.norm()) > ZERO_THRESH || fabs(k.dot(p1) - k.dot(p2)) > ZERO_THRESH;
}

int main() {
  Eigen::Matrix<double, 3, 1> p1;
  Eigen::Matrix<double, 3, 1> p2; 
  Eigen::Matrix<double, 3, 1> k;
  double theta;
  int ls_input, op_input;

  std::cout << "LS (0) or non-LS (1)?" << std::endl;

  std:: cin >> ls_input;

  std::cout << "# of Operations?" << std::endl;

  std:: cin >> op_input;

  clock_t begin = clock();

  if (ls_input == 1) {
		for (int i = 0; i < op_input; i++) {
    	sp1_setup(p1, p2, k, theta);
		}
  } else {
		for (int i = 0; i < op_input; i++) {
    	sp1_setup_LS(p1, p2, k, theta);
		}
  }

  sp1_run(p1, p2, k, theta);

  clock_t end = clock();
  double timeSec = (end - begin) / static_cast<double>( CLOCKS_PER_SEC );

  std::cout << "===== \n time (seconds): " << timeSec << std::endl;
	return 0;
}