//---------------------------------------------------------------//
// Name: subproblem_demo.cpp
// Author: Amar Maksumic
// Date: 04/17/2022
// Purpose: Demos of how to use subproblems 1 through 6
//---------------------------------------------------------------//

// import all subproblems
#include "IK/IK_3_parallel_2_intersecting.cpp"
#include "IK/IK_spherical_2_intersecting.cpp"
#include "IK/IK_spherical_2_parallel.cpp"
#include "rand_cpp.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

void IK3p2i_setup();
void IKs2i_setup();
void IKs2p_setup();

std::string to_string(std::vector<double> vec) {
  std::string str = "";
  for (unsigned int i = 0; i < vec.size(); i++) {
    str += std::to_string(vec[i]) + ", ";
  }
  return str;
}

void IK3p2i_demo() {

}