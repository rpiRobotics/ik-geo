//---------------------------------------------------------------//
// Name: IK_spherical_2_intersecting.cpp
// Author: Amar Maksumic
// Date: 02/10/2022
// Purpose: Port of the IK_spherical_2_intersecting files
//---------------------------------------------------------------//


#include <eigen3/Eigen/Dense>
#include "+IK_setups/IK_spherical_2_intersecting.h"
#include "../rand_cpp.h"
#include "../+subproblem_setups/sp_1.h"
#include "../+subproblem_setups/sp_2.h"
#include "../+subproblem_setups/sp_3.h"
#include "../+subproblem_setups/sp_4.h"

void setup(Eigen::Matrix<double, 3, 7>& H, Eigen::Matrix<double, 3, 7>& P, 
           Eigen::Matrix<double, 6, 1>& Q, Eigen::Matrix<double, 1, 6>& joint_type) {
  Eigen::Vector3d zv;
  zv << 0, 0, 0;

  Eigen::Matrix<double, 6, 1> Q = rand_angle(6);

  Eigen::Matrix<double, 1, 6> joint_type = Eigen::Matrix<double, 1, 6>::Zero();

  Eigen::Matrix<double, 3, 7> H = rand_normal_vec(7);
  Eigen::Matrix<double, 3, 7> P;
  P.col(0) = rand_vec();
  P.col(1) = zv;
  P.col(2) = rand_vec();
  P.col(3) = rand_vec();
  P.col(4) = zv;
  P.col(5) = zv;
  P.col(6) = rand_vec();
}

void setup_LS(Eigen::Matrix<double, 3, 7>& H, Eigen::Matrix<double, 3, 7>& P, 
           Eigen::Matrix<double, 6, 1>& Q, Eigen::Matrix<double, 1, 6>& joint_type) {
  Eigen::Vector3d zv;
  zv << 0, 0, 0;

  Eigen::Matrix<double, 6, 1> Q = rand_angle(6);

  Eigen::Matrix<double, 1, 6> joint_type = Eigen::Matrix<double, 1, 6>::Zero();

  Eigen::Matrix<double, 3, 7> H = rand_normal_vec(7);
  Eigen::Matrix<double, 3, 7> P;
  P.col(0) = rand_vec();
  P.col(1) = zv;
  P.col(2) = rand_vec();
  P.col(3) = rand_vec();
  P.col(4) = zv;
  P.col(5) = zv;
  P.col(6) = zv;
  H.col(1) = rand_perp_normal_vec(H.col(1));
  H.col(2) = H.col(1);
  P.col(3) = P.col(3) - H.col(2) * (H.col(2).transpose() * (P.col(2) + P.col(3)));
  P.col(4) = rand_perp_normal_vec(H.col(3));
  P.col(5) = rand_perp_normal_vec(H.col(4));

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R = rot(rand_normal_vec(), rand_angle());
  Eigen::Matrix<double, 3, 1> T = rand_vec();
}

void error() {
  //implement
}

void IK_spherical_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kin& kin, 
                                 Eigen::Matrix<double, 6, Eigen::Dynamic>& Q, Eigen::Matrix<double, 5, Eigen::Dynamic>& Q_LS) {
  //////

  Eigen::Matrix<double, 3, 1> p_16 = (p_0T - R_0T * kin.P.col(6) - kin.P.col(0)); 

  std::vector <double> theta;

  bool t3_is_ls = sp_3(kin.P.col(3), -kin.P.col(2), kin.H.col(2), p_16.norm(), theta);

  for (int i = 0; i < theta.size(); i++) {

    double q3 = theta[i];

    std::vector<double> t1, t2;

    bool t12_is_ls = sp2_run(p_16,
                             kin.P.col(2) + rot(kin.H.col(2), q3) * kin.P.col(3),
                             -kin.H.col(0),
                             kin.H.col(1),
                             t1, t2);
    for (int j = 0; j < t1.size(); j++) {
      double q1 = t1[j];
      double q2 = t2[j];

      Eigen::Matrix<double, 3, 3> R_36 = rot(-kin.H.col(2), q3) * rot(-kin.H.col(1), q2) * rot(-kin.H.col(0), q1) * R_0T;

      std::vector<double> t5;

      bool q5_is_ls = sp4_run(kin.H.col(3), kin.H.col(5), kin.H.col(4), 
                              kin.H.col(3).transpose() * R_36 * kin.H.col(5), 
                              t5);
      for (int k = 0; k < t5.size(); k++) {
        double q5 = t5[k];

        double q4, q6 = 0;

        bool q4_is_ls = sp1_run(rot(kin.H.col(4), q5) * kin.H.col(5),
                                R_36*kin.H.col(5),
                                kin.H.col(4), q4);

        bool q6_is_ls = sp1_run(rot(-kin.H.col(4), q5) * kin.H.col(3),
                                R_36.transpose()*kin.H.col(3),
                                -kin.H.col(6), q6);
        
        Eigen::Matrix<double, 6, 1> q;
        q << q1, q2, q3, q4, q5, q6;
        Q << q;

        Eigen::Matrix<double, 5, 1> q_ls;
        q_ls << t3_is_ls, t12_is_ls, q5_is_ls, q4_is_ls, q6_is_ls;
        Q_LS << q_ls;
      }
    }
  }
}