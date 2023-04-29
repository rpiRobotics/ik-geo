#include "+subproblem_setups/sp_1.h"
#include <vector>

//Add calls for each subproblem.

int main(){
   srand(time(NULL));
   Eigen::Vector3d p1, p2, k;
   double theta, error;
   bool LS;

   for(unsigned int i = 0; i < 10000; i++){
      sp1_setup(p1, p2, k, theta);
      LS = sp1_run(p1, p2, k, theta);
      error = sp1_error(p1, p2, k, theta);
      
      std::cout << LS << " " << error << std::endl;
   }
}