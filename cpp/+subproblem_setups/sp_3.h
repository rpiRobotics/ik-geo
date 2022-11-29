//---------------------------------------------------------------//
// Name: sp_3.h
// Author: Runbin Chen
// Date: 11/06/2022
//---------------------------------------------------------------//

#ifndef __sp_3_h_
#define __sp_3_h_

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../rand_cpp.h"

using namespace Eigen;

void sp_3_setup(Vector3d &p1, Vector3d &p2, Vector3d &k, double &d);

bool sp_3(Vector3d &p1, Vector3d &p2, Vector3d &k, double &d, 
		  std::vector<double> &theta);

#endif