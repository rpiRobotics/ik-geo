//---------------------------------------------------------------//
// Name: IK_spherical_2_parallel.cpp
// Author: Runbin Chen
// Date: 04/01/2023
//---------------------------------------------------------------//


/**
 * TODO:
 *   1. verify the correctness of error_given_q1
 *   2. implement search_1D function
 */

#pragma GCC optimize(3)

#include <chrono>
#include "IK_2_parallel.h"
#include "../read_csv.h"
#include "../subproblems/sp.cpp"

// purpose: general forward kinematics for serial chain robot
void fwdkin(Kin& kin, std::vector<double>& theta, 
			Eigen::Matrix<double, 3, 3>& R, Eigen::Vector3d& p) {

    p = kin.P.block<3, 1>(0, 0);
    R = Eigen::Matrix3d::Identity(3, 3);

    for (int i = 0; i < (int)kin.joint_type.size(); i ++ ) {
    	if (kin.joint_type[i] == 0 || kin.joint_type[i] == 2) {
    		// MATLAB: R = R*rot(kin.H(:,i),theta(i));
    		R = R * rot(kin.H.block<3, 1>(0, i), theta[i]);
    	}
    	else if (kin.joint_type[i] == 1 || kin.joint_type[i] == 3) {
    		// MATLAB: p = p + R*kin.H(:,i)*theta(i);
    		p = p + R * kin.H.block<3, 1>(0, i) * theta[i];
    	}
    	// MATLAB: p = p + R*kin.P(:,i+1);
    	p = p + R * kin.P.block<3, 1>(0, i + 1);
    }
}

void IK_2_parallel_setup(P &p, Soln& soln) {
	Eigen::Vector3d zv;
	zv << 0, 0, 0;

	soln.Q.clear();
	for (int i = 0; i < 6; i ++ ) {
		soln.Q.push_back(std::vector<double>());
		soln.Q[i].push_back(rand_angle());
	}
	p.kin.joint_type.clear();
	for (int i = 0; i < 6; i ++ ) {
		p.kin.joint_type.push_back(0);
	}

	p.kin.H = rand_normal_vec(6);
	p.kin.H(0, 2) = p.kin.H(0, 1);
	p.kin.H(1, 2) = p.kin.H(1, 1);
	p.kin.H(2, 2) = p.kin.H(2, 1);

	p.kin.P << rand_vec(), rand_vec(), rand_vec(), rand_vec(), zv, zv, rand_vec();

	std::vector<double> tmp;
	for (int i = 0; i < 6; i ++ )
		tmp.push_back(soln.Q[i][0]);

	fwdkin(p.kin, tmp, p.R, p.T);
}

void search_1D(void (*fun)(double, std::vector<double>, std::vector<double>, std::vector<double>), 
	double x1, double x2, int N, std::vector<double>& q1_vec, std::vector<int>& soln_num_vec);

void error_given_q1(double q1, const Kin& kin, const Eigen::Matrix<double, 3, 3>& R_06, 
	const Eigen::Matrix<double, 3, 3>& R_01, const Eigen::Vector3d& p_16, 
	std::vector<double>& e, std::vector<double>& t4, std::vector<double>& t6) {
	e.clear();
	Eigen::Vector3d h2 = kin.H.block<3, 1>(0, 1);
	Eigen::Vector3d h1 = (h2.transpose() * R_01.transpose() * R_06).transpose();
	Eigen::Vector3d h3 = h1;
	Eigen::Vector3d h4 = h2; // no negative here
	Eigen::Matrix<double, 3, 4> sp_H, sp_K, sp_P;
	sp_H << h1, h2, h3, h4;
	sp_K << -kin.H.block<3, 1>(0, 5), kin.H.block<3, 1>(0, 3), -kin.H.block<3, 1>(0, 5), kin.H.block<3, 1>(0, 3);
	sp_P << kin.P.block<3, 1>(0, 5), kin.P.block<3, 1>(0, 4), kin.H.block<3, 1>(0, 4), -kin.H.block<3, 1>(0, 4);
	double d1 = (kin.H.block<3, 1>(0, 1).transpose() * (R_01.transpose() * p_16 
		- kin.P.block<3, 1>(0, 1) - kin.P.block<3, 1>(0, 2) - kin.P.block<3, 1>(0, 3)))(0, 0);
	double d2 = 0;
	IKS::sp6_run(sp_P, sp_K, sp_H, d1, d2, t6, t4);
	for (unsigned int i_46 = 0; i_46 < t4.size(); i_46 ++ ) {
		Eigen::Matrix<double, 3, 3> R_34 = rot(kin.H.block<3, 1>(0, 3), t4[i_46]);
		Eigen::Matrix<double, 3, 3> R_56 = rot(kin.H.block<3, 1>(0, 5), t6[i_46]);
		double t23;
		bool t23_is_LS = IKS::sp1_run(
			R_34 * kin.H.block<3, 1>(0, 4), 
			R_01.transpose()*R_06*R_56.transpose()*kin.H<3, 1>(0, 4), 
			H.block<3, 1>(0, 1), 
			t23);
		Eigen::Matrix<double, 3, 3> R_13 = rot(kin.H.block<3, 1>(0, 1), t23);

		e.push_back((R_01.transpose() * p_16 - kin.P.block<3, 1>(0, 1) - R_13 * kin.P.block<3, 1>(0, 3)
		 - R_13 * R_34 * kin.P.block<3, 1>(0, 4) - R_01.transpose() * R_06 * R_56.transpose() * kin.P.block<3, 1>(0, 5)).norm()
		 - kin.P.block<3, 1>(0, 2).norm());
	}
}

Soln IK_2_parallel(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kin& kin) {
	Soln soln;
	soln.Q.clear();
	for (int i = 0; i < 6; i ++ ) 
		soln.Q.push_back(std::vector<double>());
	soln.is_LS_vec.clear();

	Eigen::Vector3d p_16 = p_0T - kin.P.block<3, 1>(0, 0) - R_06 * kin.P.block<3, 1>(0, 6);

	// need to implement search1D function
	// % [q1_vec, soln_num_vec] = search_1D(@error_given_q1, -pi, pi, 1000, true);
	std::vector<double> q1_vec;
	std::vector<int> soln_num_vec;
	search_1D(error_given_q1, -pi, pi, 1000, q1_vec, soln_num_vec);

	for (unsigned int i_q1 = 0; i_q1 < q1_vec.size(); i_q1 ++ ) {
		double q1 = q1_vec[i_q1];
		
		Eigen::Matrix<double, 3, 3> R_01 = rot(kin.H.block<3, 1>(0, 0), q1);
		
		std::vector<double> e_vec, t4, t6;
		error_given_q1(q1, e_vec, t4, t6);
		double e_i = e_vec[soln_num_vec[i_q1]];
		double q4 = t4[soln_num_vec[i_q1]];
		double q6 = t6[soln_num_vec[i_q1]];
		Eigen::Matrix<double, 3, 3> R_34 = rot(kin.H.block<3, 1>(0, 3), q4);
		Eigen::Matrix<double, 3, 3> R_56 = rot(kin.H.block<3, 1>(0, 5), q6);
	}


	return soln; 
}

int main(int argc, char * argv[]) {
	std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("IK_2_parallel.csv");
  	if (data.size() != 57) {
    	std::cerr << "Invalid input data for IK_2_parallel. \n";
    	return 0;
  	}

  	double time_avg = 0;

  	for (int i = 0; i < (int)data[0].second.size(); i ++ ) {
  		Kin kin;
  		kin.H << data[0].second[i], data[3].second[i], data[6].second[i], 
  			data[9].second[i], data[12].second[i], data[15].second[i], 
  			data[1].second[i], data[4].second[i], data[7].second[i], 
  			data[10].second[i], data[13].second[i], data[16].second[i], 
  			data[2].second[i], data[5].second[i], data[8].second[i], 
  			data[11].second[i], data[14].second[i], data[17].second[i];
  		kin.P << data[18].second[i], data[21].second[i], data[24].second[i], data[27].second[i], 
  			data[30].second[i], data[33].second[i], data[36].second[i], 
  			data[19].second[i], data[22].second[i], data[25].second[i], data[28].second[i], 
  			data[31].second[i], data[34].second[i], data[37].second[i], 
  			data[20].second[i], data[23].second[i], data[26].second[i], data[29].second[i], 
  			data[32].second[i], data[35].second[i], data[38].second[i];
  		Eigen::Matrix<double, 3, 3> R_0T;
  		R_0T << data[39].second[i], data[42].second[i], data[45].second[i], 
  			 data[40].second[i], data[43].second[i], data[46].second[i], 
  			 data[41].second[i], data[44].second[i], data[47].second[i];
  		Eigen::Vector3d p_0T;
  		p_0T << data[48].second[i], data[49].second[i], data[50].second[i];

  		auto start = std::chrono::steady_clock::now();
  		Soln soln = IK_spherical_2_parallel(R_0T, p_0T, kin);
  		auto end = std::chrono::steady_clock::now();
  		time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

  		// if (i >= 1) continue;
  		// printf("\n%d\n", i);
  		// if (soln.Q.size() == 0) {
  		// 	printf("Q is empty.\n");
  		// 	continue;
  		// }
  		// for (int i = 0; i < (int)soln.Q[0].size(); i ++ ) {
  		// 	printf("%lf %lf %lf %lf %lf %lf\n", 
  		// 		soln.Q[0][i], soln.Q[1][i], soln.Q[2][i], soln.Q[3][i], soln.Q[4][i], soln.Q[5][i]);
  		// }
  	}
  	time_avg /= (int)data[0].second.size();
  	std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl;

  	return 0;
}