//---------------------------------------------------------------//
// Name: sp_3.cpp
// Author: Runbin Chen
// Date: 11/06/2022
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include "sp_3.h"

using namespace Eigen;

int main(int argc, char* argv[]) {

	Vector3d p1, p2, k;
	double d;
	std::vector<double> theta;

	int num = 0;

	for (int arg = 1; arg < argc; arg ++ ) {
		if (argv[arg] == std::string("-n")) {
			num = atoi(argv[ ++ arg]);
		}
	}

	sp_3_setup(p1, p2, k, d);

	clock_t begin = clock();

	for (int i = 0; i < num; i ++ ) {
		// sp_3_setup(p1, p2, k, d);
		sp_3(p1, p2, k, d, theta);
	}

	clock_t end = clock();

	double timeSec = (end - begin) / static_cast<double>( CLOCKS_PER_SEC );

	std::cout << "===== \n time (seconds): " << timeSec << std::endl;

	return 0;
}