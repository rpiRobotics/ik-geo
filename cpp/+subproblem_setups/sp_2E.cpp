//---------------------------------------------------------------//
// Name: sp_2E.cpp
// Author: Runbin Chen
// Date: 10/25/2022
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <chrono>
#include "sp_2E.h"
#include "../read_csv.h"

int main(int argc, char* argv[]) {
	std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("sp_2E.csv");
  	if (data.size() != 17) {
    	std::cerr << "Invalid input data. \n";
    	return 0;
  	}

  	double time_avg = 0;

  	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
	  	Eigen::Vector3d p0, p1, p2, k1, k2;
	   	double theta1, theta2;
		p0 << data[0].second[i], data[1].second[i], data[2].second[i];
		p1 << data[3].second[i], data[4].second[i], data[5].second[i];
		p2 << data[12].second[i], data[13].second[i], data[14].second[i];
		k1 << data[6].second[i], data[7].second[i], data[8].second[i];
	    k2 << data[9].second[i], data[10].second[i], data[11].second[i];
	    theta1 = data[15].second[i];
	    theta2 = data[16].second[i];

	    auto start = std::chrono::steady_clock::now();

	    sp_2E(p0, p1, p2, k1, k2, theta1, theta2);

	    auto end = std::chrono::steady_clock::now();

	    if (!i) continue;

	    time_avg += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	    // std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
    }

  	time_avg /= (int)data[0].second.size() - 1;

  	std::cout << "===== \n time (microseconds): " << time_avg << std::endl;

	return 0;
}