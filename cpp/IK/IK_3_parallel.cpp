//---------------------------------------------------------------//
// Name: IK_3_parallel.cpp
// Author: Runbin Chen
// Date: 03/10/2023
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <chrono>
#include "IK_3_parallel.h"
#include "../read_csv.h"
#include <pthread.h>
#include "../subproblems/sp.cpp"

bool parallel_mode = false;
#ifdef PARALLEL
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

typedef struct {
	Soln * soln;
	double q_1, q_5;
	Eigen::Vector3d * p_16;
	Eigen::Matrix<double, 3, 6> * H;
	Eigen::Matrix<double, 3, 7> * P;
	const Eigen::Matrix<double, 3, 3> * R_06;
} state_t;


// wrap [rad] angle to [-PI, PI)
inline double wrapToPi(double rad) {
	return fmod(2 * M_PI + fmod(rad + M_PI, 2 * M_PI), 2 * M_PI) - M_PI;
}

// for parallel mode
void * routine(void * arg) {
	state_t * state = (state_t *)arg;
	double q_1 = state->q_1, q_5 = state->q_5;
	Eigen::Matrix<double, 3, 6> H = *(state->H);
	Eigen::Matrix<double, 3, 7> P = *(state->P);

	// solve for R_14
	double theta_14;
	bool theta_14_is_LS = IKS::sp1_run(
		rot(H.block<3, 1>(0, 4), q_5) * H.block<3, 1>(0, 5), 
		rot(H.block<3, 1>(0, 0), -q_1) * (*(state->R_06)) * H.block<3, 1>(0, 5), 
		H.block<3, 1>(0, 1), 
		theta_14);

	// solve for q3
	Eigen::Matrix<double, 3, 3> R_01 = rot(H.block<3, 1>(0, 0), q_1);
	Eigen::Matrix<double, 3, 3> R_45 = rot(H.block<3, 1>(0, 4), q_5);
	Eigen::Matrix<double, 3, 3> R_14 = rot(H.block<3, 1>(0, 1), theta_14);
	Eigen::Vector3d p_12 = P.block<3, 1>(0, 1);
	Eigen::Vector3d p_23 = P.block<3, 1>(0, 2);
	Eigen::Vector3d p_34 = P.block<3, 1>(0, 3);
	Eigen::Vector3d p_45 = P.block<3, 1>(0, 4);
	Eigen::Vector3d p_56 = P.block<3, 1>(0, 5);
	Eigen::Vector3d d_inner = R_01.transpose()*(*(state->p_16)) - p_12 - R_14*R_45*p_56 - R_14*p_45;
	double d = d_inner.norm();
	std::vector<double> theta_3;
	bool theta_3_is_LS = IKS::sp3_run(-p_34, p_23, H.block<3, 1>(0, 1), d, theta_3);

	// for q_3 = theta_3
	for (double q_3 : theta_3) {
		// % solve for q2
    	// [q_2, q_2_is_LS] = subproblem.sp_1(p_23 + rot(H(:,2), q_3)*p_34, d_inner, H(:,2));

		// solve for q2
		double q_2;
		bool q2_is_LS = IKS::sp1_run(
			p_23 + rot(H.block<3, 1>(0, 1), q_3)*p_34, 
			d_inner, 
			H.block<3, 1>(0, 1), 
			q_2);

		// q4 by subtraction
		double q_4 = wrapToPi(theta_14 - q_2 - q_3);

		// And finally q6 using rotation component
        double q_6;
        bool q_6_is_LS = IKS::sp1_run(
        	H.block<3, 1>(0, 4), 
        	R_45.transpose() * R_14.transpose() * R_01.transpose() * (*(state->R_06)) * H.block<3, 1>(0, 4), 
        	H.block<3, 1>(0, 5), 
        	q_6);

		#ifdef PARALLEL
        pthread_mutex_lock(&mutex);
        #endif
        { // critical segments
	        (*(state->soln)).Q[0].push_back(q_1);
	        (*(state->soln)).Q[1].push_back(q_2);
	        (*(state->soln)).Q[2].push_back(q_3);
	        (*(state->soln)).Q[3].push_back(q_4);
	        (*(state->soln)).Q[4].push_back(q_5);
	        (*(state->soln)).Q[5].push_back(q_6);

	        // is_LS_vec = [is_LS_vec theta_14_is_LS||theta_3_is_LS||q_2_is_LS||q_6_is_LS];
	        (*(state->soln)).is_LS_vec[0].push_back(theta_14_is_LS);
	        (*(state->soln)).is_LS_vec[1].push_back(theta_3_is_LS);
	        (*(state->soln)).is_LS_vec[2].push_back(q2_is_LS);
	        (*(state->soln)).is_LS_vec[3].push_back(q_6_is_LS);
        }
        #ifdef PARALLEL
        pthread_mutex_unlock(&mutex);
        #endif
	}

	long * return_value = new long;
	*return_value = 0;

	delete state;

	pthread_exit( return_value );
	return NULL;
}

void fwdkin(Kin& kin, std::vector<double>& theta, 
			Eigen::Matrix<double, 3, 3>& R, Eigen::Vector3d& p) {

    p = kin.P.block<3, 1>(0, 0);
    R = Eigen::Matrix3d::Identity(3, 3);

    for (int i = 0; i < (int)kin.joint_type.size(); i ++ ) {
    	if (kin.joint_type[i] == 0 || kin.joint_type[i] == 2) {
    		R = R * rot(kin.H.block<3, 1>(0, i), theta[i]);
    	}
    	else if (kin.joint_type[i] == 1 || kin.joint_type[i] == 3) {
    		p = p + R * kin.H.block<3, 1>(0, i) * theta[i];
    	}
    	p = p + R * kin.P.block<3, 1>(0, i + 1);
    }
}

void IK_3_parallel_setup(P &p, Soln& soln) {
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

Soln IK_3_parallel(const Eigen::Matrix<double, 3, 3>& R_06, const Eigen::Vector3d& p_0T, const Kin& kin) {

	Eigen::Matrix<double, 3, 6> H = kin.H;
	Eigen::Matrix<double, 3, 7> P = kin.P;

	Eigen::Vector3d p_16 = p_0T - P.block<3, 1>(0, 0) - R_06 * P.block<3, 1>(0, 6);

	Eigen::Matrix<double, 3, 4> H_sp;
	H_sp << H.block<3, 1>(0, 1), H.block<3, 1>(0, 1), H.block<3, 1>(0, 1), H.block<3, 1>(0, 1);
	Eigen::Matrix<double, 3, 4> K_sp;
	K_sp << -H.block<3, 1>(0, 0), H.block<3, 1>(0, 4), -H.block<3, 1>(0, 0), H.block<3, 1>(0, 4);
	Eigen::Matrix<double, 3, 4> P_sp;
	P_sp << p_16, -P.block<3, 1>(0, 5), R_06 * H.block<3, 1>(0, 0), H.block<3, 1>(0, 4);
	double d1 = (H.block<3, 1>(0, 1).transpose() * (P.block<3, 1>(0, 2)+P.block<3, 1>(0, 3)+P.block<3, 1>(0, 4)
		+ P.block<3, 1>(0, 1)))(0, 0);
	double d2 = 0;

	std::vector<double> theta1, theta5;
	IKS::sp6_run(H_sp, K_sp, P_sp, d1, d2, theta1, theta5);

	Soln soln;
	soln.Q.clear();
	for (int i = 0; i < 6; i ++ ) 
		soln.Q.push_back(std::vector<double>());
	soln.is_LS_vec.clear();
	for (int i = 0; i < 4; i ++ ) 
		soln.is_LS_vec.push_back(std::vector<bool>());

#ifdef PARALLEL
	pthread_t tid[theta1.size()];
#endif

	for (unsigned int i = 0; i < theta1.size(); i ++ ) {
		double q_1 = theta1[i];
		double q_5 = theta5[i];

		#ifdef PARALLEL
		{
			state_t * child_arg = new state_t;
			child_arg->q_1 = q_1;
			child_arg->q_5 = q_5;
			child_arg->soln = &soln;
			child_arg->p_16 = &p_16;
			child_arg->R_06 = &R_06;
			child_arg->H = &H;
			child_arg->P = &P;
			int rc = pthread_create( &tid[i], NULL, routine, child_arg );
		    if ( rc != 0 ) {
		      	fprintf( stderr, "pthread_create() failed (%d)\n", rc );
		    }
			continue;
		}
		#endif

		// solve for R_14
		double theta_14;
		bool theta_14_is_LS = IKS::sp1_run(
			rot(H.block<3, 1>(0, 4), q_5) * H.block<3, 1>(0, 5), 
			rot(H.block<3, 1>(0, 0), -q_1) * R_06 * H.block<3, 1>(0, 5), 
			H.block<3, 1>(0, 1), 
			theta_14);

		// solve for q3
		Eigen::Matrix<double, 3, 3> R_01 = rot(H.block<3, 1>(0, 0), q_1);
		Eigen::Matrix<double, 3, 3> R_45 = rot(H.block<3, 1>(0, 4), q_5);
		Eigen::Matrix<double, 3, 3> R_14 = rot(H.block<3, 1>(0, 1), theta_14);
		Eigen::Vector3d p_12 = P.block<3, 1>(0, 1);
		Eigen::Vector3d p_23 = P.block<3, 1>(0, 2);
		Eigen::Vector3d p_34 = P.block<3, 1>(0, 3);
		Eigen::Vector3d p_45 = P.block<3, 1>(0, 4);
		Eigen::Vector3d p_56 = P.block<3, 1>(0, 5);
		Eigen::Vector3d d_inner = R_01.transpose()*p_16 - p_12 - R_14*R_45*p_56 - R_14*p_45;
		double d = d_inner.norm();
		std::vector<double> theta_3;
		bool theta_3_is_LS = IKS::sp3_run(-p_34, p_23, H.block<3, 1>(0, 1), d, theta_3);

		// for q_3 = theta_3
		for (double q_3 : theta_3) {
			// % solve for q2
        	// [q_2, q_2_is_LS] = subproblem.sp_1(p_23 + rot(H(:,2), q_3)*p_34, d_inner, H(:,2));

			// solve for q2
			double q_2;
			bool q2_is_LS = IKS::sp1_run(
				p_23 + rot(H.block<3, 1>(0, 1), q_3)*p_34, 
				d_inner, 
				H.block<3, 1>(0, 1), 
				q_2);

			// q4 by subtraction
			double q_4 = wrapToPi(theta_14 - q_2 - q_3);

			// And finally q6 using rotation component
	        double q_6;
	        bool q_6_is_LS = IKS::sp1_run(
	        	H.block<3, 1>(0, 4), 
	        	R_45.transpose() * R_14.transpose() * R_01.transpose() * R_06 * H.block<3, 1>(0, 4), 
	        	H.block<3, 1>(0, 5), 
	        	q_6);

	        soln.Q[0].push_back(q_1);
	        soln.Q[1].push_back(q_2);
	        soln.Q[2].push_back(q_3);
	        soln.Q[3].push_back(q_4);
	        soln.Q[4].push_back(q_5);
	        soln.Q[5].push_back(q_6);

	        // is_LS_vec = [is_LS_vec theta_14_is_LS||theta_3_is_LS||q_2_is_LS||q_6_is_LS];
	        soln.is_LS_vec[0].push_back(theta_14_is_LS);
	        soln.is_LS_vec[1].push_back(theta_3_is_LS);
	        soln.is_LS_vec[2].push_back(q2_is_LS);
	        soln.is_LS_vec[3].push_back(q_6_is_LS);
		}
	}

#ifdef PARALLEL
	for (unsigned int i = 0; i < theta1.size(); i ++ ) {
		long * status;
		pthread_join( tid[i], (void **)&status );
		delete status;
	}
#endif

	return soln; 
}


/**
 * compile from terminal/shell as follows:
 * 
 *   bash$ g++ IK_3_parallel.cpp ../+subproblem_setups/sp.cpp -o IK_3_parallel.out -O3
 * 
 * to turn on the parallel mode, compile it as follows:
 * 
 *   bash$ g++ IK_3_parallel.cpp ../+subproblem_setups/sp.cpp -o IK_3_parallel.out -O3 -D PARALLEL -pthread
 * 
 * run the executable:
 * 
 *   bash$ ./IK_3_parallel.out
 * 
 * currently the parallel mode is much more time-consuming due to its overhead of about 9 ms. 
 * I reserved it here as an example of how we could apply parallel programming technology. 
 * It might be useful in the future. 
 */

// The main function is for debugging and should be removed later.
// Can put the timing test code here for quick checking 
//  and later move all the timing test code to a single timing test cpp file.
int main(int argc, char * argv[]) {
#ifdef PARALLEL
	parallel_mode = true;
#endif

	// std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("IK_3_parallel.csv");
	std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("../../test_cases/IK_3_parallel.csv");
  	if (data.size() != 57) {
    	std::cerr << "Invalid input data for IK_3_parallel. \n";
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
  		Soln soln = IK_3_parallel(R_0T, p_0T, kin);
  		auto end = std::chrono::steady_clock::now();
  		time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

  		// if (i >= 1) continue; // change the number here to show output for debugging
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