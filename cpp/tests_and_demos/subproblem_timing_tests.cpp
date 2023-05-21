/**
 * Use CSV compiler in matlab tests folder to generate sp_x.csv for each subproblem
 * Otherwise, this will not work.
 * 
 * timing tests of all subproblems
 * 
 * compile from terminal/shell as follows:
 * 
 *   bash$ g++ -Wall sp_tests.cpp sp.cpp -o sp_tests
 * 
 * run the executable:
 * 
 *   bash$ ./sp_tests
 * 
 */

#include <iostream>
#include <chrono>

#include "../subproblems/sp.cpp"
#include "../read_csv.h"

int main(int argc, char * argv[])
{
	std::vector<std::pair<std::string, std::vector<double>>> data;
	double time_avg;


	// subproblem 1
	std::cout << "subproblem 1:\n";
	data = read_csv("sp_1.csv");
  	if (data.size() != 10) {
	    std::cerr << "Invalid input data for subproblem 1. \n";
	    return 0;
  	}
  	time_avg = 0;
  	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
	    Eigen::Vector3d p1, k, p2;
	    double theta;
	    p1 << data[0].second[i], data[1].second[i], data[2].second[i];
	    k << data[3].second[i], data[4].second[i], data[5].second[i];
	    p2 << data[6].second[i], data[7].second[i], data[8].second[i];
	    theta = data[9].second[i];

	    auto start = std::chrono::steady_clock::now();

	    IKS::sp1_run(p1, p2, k, theta);

	    auto end = std::chrono::steady_clock::now();

	    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  	}
  	time_avg /= (int)data[0].second.size();
  	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl << std::endl;


  	// subproblem 2
	std::cout << "subproblem 2:\n";
  	data = read_csv("sp_2.csv");
  	if (data.size() != 14) {
	    std::cerr << "Invalid data for sp2.\n";
	    return 0;
  	}
  	time_avg = 0;
  	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
	    Eigen::Vector3d p1, p2, k1, k2;
	    std::vector<double> theta1, theta2;
	    p1 << data[0].second[i], data[1].second[i], data[2].second[i];
	    k1 << data[3].second[i], data[4].second[i], data[5].second[i];
	    k2 << data[6].second[i], data[7].second[i], data[8].second[i];
	    p2 << data[9].second[i], data[10].second[i], data[11].second[i];
	    theta1.push_back(data[12].second[i]);
	    theta2.push_back(data[13].second[i]);

	    auto start = std::chrono::steady_clock::now();

	    IKS::sp2_run(p1, p2, k1, k2, theta1, theta2);

	    auto end = std::chrono::steady_clock::now();

	    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  	}
  	time_avg /= (int)data[0].second.size();
  	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl << std::endl;


  	// subproblem 2E
	std::cout << "subproblem 2E:\n";
  	data = read_csv("sp_2E.csv");
  	if (data.size() != 17) {
    	std::cerr << "Invalid input data. \n";
    	return 0;
  	}
  	time_avg = 0;
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

	    IKS::sp2E_run(p0, p1, p2, k1, k2, theta1, theta2);

	    auto end = std::chrono::steady_clock::now();

	    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    }
  	time_avg /= (int)data[0].second.size();
  	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl << std::endl;


  	// subproblem 3
	std::cout << "subproblem 3:\n";
  	data = read_csv("sp_3.csv");
  	if (data.size() != 11) {
    	std::cerr << "Invalid input data for subproblem 3. \n";
    	return 0;
  	}
  	time_avg = 0;
  	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
	  	Eigen::Vector3d p1, p2, k;
	   	double d;
	   	std::vector<double> theta;
			p1 << data[0].second[i], data[1].second[i], data[2].second[i];
			p2 << data[3].second[i], data[4].second[i], data[5].second[i];
			k << data[6].second[i], data[7].second[i], data[8].second[i];
	    d = data[9].second[i];
	    theta.push_back(data[10].second[i]);

	    auto start = std::chrono::steady_clock::now();

	    IKS::sp3_run(p1, p2, k, d, theta);

	    auto end = std::chrono::steady_clock::now();

	    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    }
  	time_avg /= (int)data[0].second.size();
  	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl << std::endl;


  	// subproblem 4
	std::cout << "subproblem 4:\n";
  	data = read_csv("sp_4.csv");
	if (data.size() != 11) {
		std::cerr << "Invalid data for sp4.\n";
		return 0;
	}
	time_avg = 0;
	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
		Eigen::Vector3d p1, k1, h1;
		std::vector<double> theta;
		double d;
		p1 << data[0].second[i], data[1].second[i], data[2].second[i];
		k1 << data[3].second[i], data[4].second[i], data[5].second[i];
		h1 << data[6].second[i], data[7].second[i], data[8].second[i];
		d = data[9].second[i];
		theta.push_back(data[10].second[i]);

		auto start = std::chrono::steady_clock::now();

		IKS::sp4_run(p1, k1, h1, d, theta);

		auto end = std::chrono::steady_clock::now();

		time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
	}
	time_avg /= (int)data[0].second.size();
	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl << std::endl;


	// subproblem 5
	std::cout << "subproblem 5:\n";
	data = read_csv("sp_5.csv");
  	if (data.size() != 24) {
    	std::cerr << "Invalid input data for subproblem 5. \n";
    	return 0;
  	}
  	time_avg = 0;
  	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
	  	Eigen::Vector3d p0, p1, p2, p3, k1, k2, k3;
	   	std::vector<double> theta1, theta2, theta3;
			p1 << data[0].second[i], data[1].second[i], data[2].second[i];
			p2 << data[3].second[i], data[4].second[i], data[5].second[i];
			p3 << data[6].second[i], data[7].second[i], data[8].second[i];
			k1 << data[9].second[i], data[10].second[i], data[11].second[i];
	    k2 << data[12].second[i], data[13].second[i], data[14].second[i];
	    k3 << data[15].second[i], data[16].second[i], data[17].second[i];
			p0 << data[18].second[i], data[19].second[i], data[20].second[i];
			theta1.push_back(data[21].second[i]);
			theta2.push_back(data[22].second[i]);
			theta3.push_back(data[23].second[i]);

	    auto start = std::chrono::steady_clock::now();

	    IKS::sp5_run(p0, p1, p2, p3, k1, k2, k3, theta1, theta2, theta3);

	    auto end = std::chrono::steady_clock::now();

	    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    }
  	time_avg /= (int)data[0].second.size();
  	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl << std::endl;



  	// subproblem 6
	std::cout << "subproblem 6:\n";
  	data = read_csv("sp_6.csv");
	if (data.size() != 40) {
		std::cerr << "Invalid data for sp6.\n";
		return 0;
	}
	time_avg = 0;
	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
		Eigen::Matrix<double, 3, 4> h, k, p;
		double d1, d2;
		std::vector<double> theta1, theta2;
		h << data[0].second[i], data[1].second[i], data[2].second[i], data[3].second[i],
		     data[4].second[i], data[5].second[i], data[6].second[i], data[7].second[i],
		     data[8].second[i], data[9].second[i], data[10].second[i], data[11].second[i];
		k << data[12].second[i], data[13].second[i], data[14].second[i], data[15].second[i],
		     data[16].second[i], data[17].second[i], data[18].second[i], data[19].second[i],
		     data[20].second[i], data[21].second[i], data[22].second[i], data[23].second[i];
		p << data[24].second[i], data[25].second[i], data[26].second[i], data[27].second[i],
		     data[28].second[i], data[29].second[i], data[30].second[i], data[31].second[i],
		     data[32].second[i], data[33].second[i], data[34].second[i], data[35].second[i];
		d1 = data[36].second[i];
		d2 = data[37].second[i];
		theta1.push_back(data[38].second[i]);
		theta2.push_back(data[39].second[i]);

		auto start = std::chrono::steady_clock::now();

		IKS::sp6_run(h, k, p, d1, d2, theta1, theta2);

		auto end = std::chrono::steady_clock::now();

		time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
	}
	time_avg /= (int)data[0].second.size();
	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl << std::endl;


  	return 0;
}