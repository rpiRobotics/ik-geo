#include <stdlib.h>
#include <time.h>
#include <math.h>


//Global variables for functions
double pi = 2 * acos(0.0); 

//Create number from 0.0000 to 1.0000
double rand_0to1(){
	srand(time(NULL));
   return (rand() % 10001) / 10000;
}



double hat(){

}

//Create a random angle matrix
//TODO: Needs to be adjusted to use size.  Matrix of size X size
double rand_angle(int size = 1){
   double angle = rand_0to1();
   return angle*2*pi - pi;
}

//Final type will not be a double, but whatever form we choose for matrices
//This applies to all vec functions below
double rand_normal_vec(int size = 1){

}

double rand_perp_normal_vec(/*Vector input*/){

}

double rand_vec(int size = 1){

}

//Create 3x3 rotation matrix using the Euler Rodrigues formula
double rot(double k, double theta){

}