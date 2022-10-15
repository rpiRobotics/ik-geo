#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include<iomanip>


//Global variables for functions
double pi = 2 * acos(0.0);

//Create number from 0.0000 to 1.0000
double rand_0to1(){
   return double(rand() % 10001) / 10000;
}

//Create a random angle
double rand_angle(){
   return rand_0to1()*2*pi - pi;
}

//Create random vector of height = 3
Eigen::Vector3d rand_vec(){
   Eigen::Vector3d vec;
   vec << rand_0to1(), rand_0to1(), rand_0to1();
   return vec;
}

Eigen::Vector3d rand_normal_vec(){
   Eigen::Vector3d vec = rand_vec();
   vec /= vec.norm();
   return vec;
}

//Only works with input vectors of size 1.  Same as MATLAB implementation
Eigen::Vector3d rand_perp_normal_vec(const Eigen::Vector3d& vec){
   Eigen::Vector3d randCross = rand_vec();
   randCross = randCross.cross(vec);
   return randCross/randCross.norm();
}

// HAT: khat = hat(k)
Eigen::Matrix3d hat(const Eigen::Vector3d &k) {
	Eigen::Matrix3d khat(3, 3);
	khat << 0,  -k(2), k(1),
		    k(2),  0, -k(0),
		   -k(1), k(0), 0;
	return khat;
}

// ROT: R = rot(k, theta)
Eigen::Matrix3d rot(Eigen::Vector3d k, double theta) {
	k = k / k.norm();
	Eigen::Matrix3d R(3, 3);
	R = Eigen::Matrix3d::Identity(3, 3) + sin(theta)*hat(k) + (1-cos(theta))*hat(k)*hat(k);
	return R;
}