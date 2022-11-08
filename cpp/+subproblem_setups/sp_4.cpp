//---------------------------------------------------------------//
// Name: sp_4.cpp
// Author: Amar Maksumic
// Date: 11/01/2022
// Purpose: Port of the subproblem/sp_4.m file
//---------------------------------------------------------------//

#include <iostream>
#include "sp_4.h"

using namespace Eigen;

int main() {
  Eigen::Matrix<double, 3, 1> p, k, h;
  Eigen::Matrix<double, 2, 1> theta;
  double d;
  int ls_input, op_input;

  std::cout << "LS (0) or non-LS (1)?" << std::endl;

  std:: cin >> ls_input;

  std::cout << "# of Operations?" << std::endl;

  std:: cin >> op_input;

  double solve_time = 0;

	for (int i = 0; i < (op_input/100); i++) {
    if (ls_input == 1) {
      sp4_setup(p, k, h, d, theta);
    } else {
      sp4_setup_LS(p, k, h, d, theta);
    }
    clock_t begin = clock();
    for (int j = 0; j < 100; j++) {
  	  sp4_run(p, k, h, d, theta);
    }
    clock_t end = clock();
    solve_time += (end - begin);
	}
  
  double timeSec = solve_time / static_cast<double>( CLOCKS_PER_SEC );

  std::cout << "===== \n time (seconds): " << timeSec << std::endl;
	return 0;
}