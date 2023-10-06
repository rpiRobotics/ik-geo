//---------------------------------------------------------------//
// Name: IK_spherical_2_parallel.cpp
// Author: Runbin Chen
// Date: 02/01/2023
//---------------------------------------------------------------//

#include "IK_spherical_2_parallel.h"
#include "../subproblems/sp.h"

Solution IK_spherical_2_parallel(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kinematics& kin) {
	Solution soln;

	// Use subproblem 4 to find up to two solutions for q1
	std::vector<double> t1(1, 1.0);
	bool q1_is_LS = IKS::sp4_run(p_0T - R_0T * kin.P.block<3, 1>(0, 6) - kin.P.block<3, 1>(0, 0),
								 -kin.H.block<3, 1>(0, 0),
								 kin.H.block<3, 1>(0, 1),
								 (kin.H.block<3, 1>(0, 1).transpose()*(kin.P.block<3, 1>(0, 1)+
																	   kin.P.block<3, 1>(0, 2)+kin.P.block<3, 1>(0, 3)))(0, 0), t1);


	// Use subproblem 3 to find up to two solutions for q3
	// for q1 = t1
	for (unsigned int q1_i = 0; q1_i < t1.size(); q1_i ++ ) {
		double q1 = t1[q1_i];
		std::vector<double> t3;
		bool q3_is_LS = IKS::sp3_run(-kin.P.block<3, 1>(0, 3),
									 kin.P.block<3, 1>(0, 2),
									 kin.H.block<3, 1>(0, 2),
									 (rot(-kin.H.block<3, 1>(0, 0), q1)*(-p_0T+R_0T*kin.P.block<3, 1>(0, 6)+kin.P.block<3, 1>(0, 0))+
									  kin.P.block<3, 1>(0, 1)).norm(), t3);


		// Solve for q2 using subproblem 1
		// for q3 = t3
		for (double q3 : t3) {
			double q2;
			bool q2_is_LS = IKS::sp1_run(-kin.P.block<3, 1>(0, 2) - rot(kin.H.block<3, 1>(0, 2), q3)*kin.P.block<3, 1>(0, 3),
										 rot(-kin.H.block<3, 1>(0, 0), q1)*(-p_0T+R_0T*kin.P.block<3, 1>(0, 6)+kin.P.block<3, 1>(0, 0))+
										 kin.P.block<3, 1>(0, 1),
										 kin.H.block<3, 1>(0, 1),
										 q2);

			Eigen::Matrix<double, 3, 3> R_36 = rot(-kin.H.block<3, 1>(0, 2), q3) *
				rot(-kin.H.block<3, 1>(0, 1), q2) *
				rot(-kin.H.block<3, 1>(0, 0), q1) * R_0T; // R_T6 = 0

			// Solve for q5 using subproblem 4
			std::vector<double> t5(1, 1.0);
			bool q5_is_LS = IKS::sp4_run(kin.H.block<3, 1>(0, 5),
										 kin.H.block<3, 1>(0, 4),
										 kin.H.block<3, 1>(0, 3),
										 (kin.H.block<3, 1>(0, 3).transpose()*R_36*kin.H.block<3, 1>(0, 5))(0, 0),
										 t5);

			// Solve for q4 using subproblem 1
			// for q5 = t5
			for (unsigned int q5_i = 0; q5_i < t5.size(); q5_i ++ ) {
				double q5 = t5[q5_i];
				double q4;
				bool q4_is_LS = IKS::sp1_run(rot(kin.H.block<3, 1>(0, 4), q5) * kin.H.block<3, 1>(0, 5),
											 R_36 * kin.H.block<3, 1>(0, 5),
											 kin.H.block<3, 1>(0, 3),
											 q4);

				// Solve for q6 using subproblem 1
				double q6;
				bool q6_is_LS = IKS::sp1_run(rot(-kin.H.block<3, 1>(0, 4), q5) * kin.H.block<3, 1>(0, 3),
											 R_36.transpose() * kin.H.block<3, 1>(0, 3),
											 -kin.H.block<3, 1>(0, 5),
											 q6);

				Eigen::Matrix<double, 6, 1> q_i;
				q_i << q1, q2, q3, q4, q5, q6;
				soln.q.push_back(q_i);
				soln.is_ls.push_back(q1_is_LS||q2_is_LS||q3_is_LS||q4_is_LS||q5_is_LS||q6_is_LS);
			}
		}
	}

	return soln;
}
