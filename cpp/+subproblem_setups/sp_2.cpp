//---------------------------------------------------------------//
// Name: sp_2.h
// Author: Amar Maksumic
// Date: 11/01/2022
// Purpose: Port of the subproblem/sp_2.m file
//---------------------------------------------------------------//

#include <iostream>
#include "sp_2.h"

using namespace Eigen;

int main() {
  Eigen::Matrix<double, 3, 1> p1, p2, k1, k2;
  double theta1, theta2;
  int ls_input, op_input;

  std::cout << "LS (0) or non-LS (1)?" << std::endl;

  std:: cin >> ls_input;

  std::cout << "# of Operations?" << std::endl;

  std:: cin >> op_input;

  double solve_time = 0;

	for (int i = 0; i < (op_input/100); i++) {
    if (ls_input == 1) {
      sp2_setup(p1, p2, k1, k2, theta1, theta2);
    } else {
      sp2_setup_LS(p1, p2, k1, k2, theta1, theta2);
    }
    clock_t begin = clock();
    for (int j = 0; j < 100; j++) {
  	  sp2_run(p1, p2, k1, k2, theta1, theta2);
    }
    clock_t end = clock();
    solve_time += (end - begin);
	}
  
  double timeSec = solve_time / static_cast<double>( CLOCKS_PER_SEC );

  std::cout << "===== \n time (seconds): " << timeSec << std::endl;
	return 0;
}