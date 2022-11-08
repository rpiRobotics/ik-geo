//---------------------------------------------------------------//
// Name: sp_2E.cpp
// Author: Runbin Chen
// Date: 10/25/2022
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include "sp_2E.h"

using namespace Eigen;

int main(int argc, char* argv[]) {

	Vector3d p0, p1, p2, k1, k2;
	double theta1, theta2;

	int num = 0;

	for (int arg = 1; arg < argc; arg ++ ) {
		if (argv[arg] == std::string("-n")) {
			num = atoi(argv[ ++ arg]);
		}
	}

	sp_2E_setup(p0, p1, p2, k1, k2, theta1, theta2);

	clock_t begin = clock();

	for (int i = 0; i < num; i ++ ) {
		// sp_2E_setup(p0, p1, p2, k1, k2, theta1, theta2);
		sp_2E(p0, p1, p2, k1, k2, theta1, theta2);
	}

	clock_t end = clock();

	double timeSec = (end - begin) / static_cast<double>( CLOCKS_PER_SEC );

	std::cout << "===== \n time (seconds): " << timeSec << std::endl;

	return 0;
}
