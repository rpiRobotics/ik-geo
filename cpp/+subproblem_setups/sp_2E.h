//---------------------------------------------------------------//
// Name: sp_2E.h
// Author: Runbin Chen
// Date: 11/05/2022
//---------------------------------------------------------------//

#ifndef __sp_2E_h_
#define __sp_2E_h_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "../rand_cpp.h"

using namespace Eigen;

void sp_2E_setup(Vector3d &p0, Vector3d &p1, Vector3d &p2, Vector3d &k1, Vector3d &k2, 
		  double &theta1, double &theta2);

void sp_2E(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2, const Vector3d &k1, const Vector3d &k2, 
		  double &theta1, double &theta2);

#endif