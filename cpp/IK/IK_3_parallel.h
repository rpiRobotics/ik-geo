//---------------------------------------------------------------//
// Name: IK_3_parallel.h
// Author: Runbin Chen
// Date: 03/10/2023
//---------------------------------------------------------------//

#ifndef __IK_3_parallel_h_
#define __IK_3_parallel_h_

#include <iostream>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>

extern double rand_0to1();
extern Eigen::Matrix<double, 3, 3> hat(const Eigen::Matrix<double, 3, Eigen::Dynamic>& vec);
extern double rand_angle();
extern Eigen::Matrix<double, Eigen::Dynamic, 1> rand_angle(int N);
extern Eigen::Matrix<double, 3, Eigen::Dynamic> rand_vec(int N = 1);
extern Eigen::Matrix<double, 3, Eigen::Dynamic> rand_normal_vec(int size = 1);
extern Eigen::VectorXd rand_perp_normal_vec(const Eigen::Vector3d& vec);
extern Eigen::Matrix<double, 3, Eigen::Dynamic> rot(Eigen::Matrix<double, 3, Eigen::Dynamic> k, double theta);

struct Kin {
	Eigen::Matrix<double, 3, 6> H;
	Eigen::Matrix<double, 3, 7> P;
	std::vector<int> joint_type;
};

struct Soln {
	std::vector<std::vector<double>> Q;
	std::vector<std::vector<bool>> is_LS_vec;
};

struct P {
	Kin kin;
	Eigen::Matrix<double, 3, 3> R;
	Eigen::Vector3d T;
};

void * routine(void * arg);

void fwdkin(Kin& kin, std::vector<double>& theta, Eigen::Matrix<double, 3, 3>& R, Eigen::Vector3d& p);

void IK_3_parallel_setup(P &p, Soln& soln);

Soln IK_3_parallel(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kin& kin);

#endif