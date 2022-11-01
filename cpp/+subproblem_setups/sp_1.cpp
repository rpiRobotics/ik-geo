//---------------------------------------------------------------//
// Name: sp_1.h
// Author: Ashton Ropp, Runbin Chen
// Date: 10/15/2022
// Purpose: Port of the subproblem/sp_1.m file
//---------------------------------------------------------------//

#include <iostream>
#include "sp_1.h"

using namespace Eigen;

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

  if (ls_input == 1) {
		sp1_setup(p1, p2, k, theta);
  } else {
		sp1_setup_LS(p1, p2, k, theta);
  }

  double solve_time = 0;

	for (int i = 0; i < op_input; i++) {
    clock_t begin = clock();
  	sp1_run(p1, p2, k, theta);
    clock_t end = clock();
    solve_time += (end - begin);
	}
  
  double timeSec = solve_time / static_cast<double>( CLOCKS_PER_SEC );

  std::cout << "===== \n time (seconds): " << timeSec << std::endl;
	return 0;
}