#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>


//Global variables for functions
double pi = 2 * acos(0.0);

//Create number from 0.0000 to 1.0000
double rand_0to1(){
	srand(time(NULL));
   return (rand() % 10001) / 10000;
}

//Example input is a normal vector
//Matrix cross-product for 3 x 3 vector
Eigen::MatrixXd hat(const Eigen::MatrixXd& vec){
   Eigen::Matrix3d output;
   output << 0, -vec(2), vec(1), vec(3), 0, -vec(0), -vec(1), vec(0), 0;
   return output;
}

//Create a random angle matrix of size NxN
Eigen::MatrixXd rand_angle(int N = 1){
   Eigen::MatrixXd vec(N, N);
   for(int i = 0; i < N; i++){
      for(int j = 0; j < 3; j++)
         vec << rand_0to1()*2*pi - pi;
   }
   return vec;
}

Eigen::MatrixXd rand_normal_vec(int size = 1){
   Eigen::MatrixXd vec = rand_vec(size);
   for(int i = 0; i < size; i++){
      //Get norm of each vector/column
      Eigen::Vector3d col;
      for(int r = 0; r < 3; r++)
         col << vec(r, i);
      
      //Then apply norm to each index
      for(int j = 0; j < 3; j++){
         vec(j, i) = vec(j, i) / col.norm();
      }
   }
}

//Only works with input vectors of size 1.  Same as MATLAB implementation
Eigen::VectorXd rand_perp_normal_vec(const Eigen::VectorXd& vec){
   Eigen::VectorXd randCross = rand_vec();
   randCross = randCross.cross(vec);
   return randCross/randCross.norm();
}

//Create random matrix of width = size, height = 3
Eigen::MatrixXd rand_vec(int N = 1){
   Eigen::MatrixXd vec(3, N);
   for(int i = 0; i < N; i++){
      for(int j = 0; j < 3; j++)
         vec << rand_0to1();
   }
   return vec;
}

//Create 3x3 rotation matrix using the Euler Rodrigues formula
//TODO: Compare notes
Eigen::MatrixXd rot(Eigen::MatrixXd k, double theta){
   Eigen::Matrix3d eye;
   eye = Eigen::Matrix3d::Identity();
   eye.setIdentity();
   k = k/k.norm();

   return eye + sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
}