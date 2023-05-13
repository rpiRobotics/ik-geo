//---------------------------------------------------------------//
// Name: IK_spherical_2_intersecting.cpp
// Author: Amar Maksumic
// Date: 02/10/2022
// Purpose: Port of the IK_spherical_2_intersecting files
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "IK_spherical_2_intersecting.h"
#include "../rand_cpp.h"
#include "../subproblems/sp.cpp"

namespace IKS {

  void fwdkin(const Kin& kin, const Soln& soln, 
              Eigen::Matrix<double, 3, 1>& p, Eigen::Matrix<double, 3, 3>& R) {
    p = kin.P.col(0);
    R = Eigen::Matrix3d::Identity();
    R.setIdentity();

    for (unsigned int i = 0; i < kin.joint_type.size(); i++) {
      if (kin.joint_type(i) == 0 || kin.joint_type(i) == 2) {
        R = R * rot(kin.H.col(i), soln.Q[i][0]);
      } else if (kin.joint_type(i) == 1 || kin.joint_type(i) == 3) {
        p = p + R * kin.H.col(i)*soln.Q[i][0];
      }
      p = p + R*kin.P.col(i + 1);
    }
  }

  void setup(Kin& kin, Soln& soln,
            Eigen::Matrix<double, 3, 1>& T, Eigen::Matrix<double, 3, 3>& R) {
    Eigen::Vector3d zv;
    zv << 0, 0, 0;

    for (int i = 0; i < 6; i++) {
      soln.Q.push_back(std::vector<double>());
      for (int j = 0; j < 5; j++) {
        soln.Q[i].push_back(rand_angle());
      }
    }


    kin.joint_type = Eigen::Matrix<double, 1, 6>::Zero();

    kin.H = rand_normal_vec(6);
    kin.P.col(0) = rand_vec();
    kin.P.col(1) = zv;
    kin.P.col(2) = rand_vec();
    kin.P.col(3) = rand_vec();
    kin.P.col(4) = zv;
    kin.P.col(5) = zv;
    kin.P.col(6) = rand_vec();


    IKS::fwdkin(kin, soln, T, R);
  }

  void setup_LS(Kin& kin, Soln& soln,
                Eigen::Matrix<double, 3, 1>& T, Eigen::Matrix<double, 3, 3>& R) {
                  
    Eigen::Vector3d zv;
    zv << 0, 0, 0;

    for (int i = 0; i < 6; i++) {
      soln.Q.push_back(std::vector<double>());
      for (int j = 0; j < 5; j++) {
        soln.Q[i].push_back(rand_angle());
      }
    }

    kin.joint_type = Eigen::Matrix<double, 1, 6>::Zero();

    kin.H = rand_normal_vec(6);
    kin.P.col(0) = rand_vec();
    kin.P.col(1) = zv;
    kin.P.col(2) = rand_vec();
    kin.P.col(3) = rand_vec();
    kin.P.col(4) = zv;
    kin.P.col(5) = zv;
    kin.P.col(6) = zv;
    kin.H.col(1) = rand_perp_normal_vec(kin.H.col(1));
    kin.H.col(2) = kin.H.col(1);
    kin.P.col(3) = kin.P.col(3) - kin.H.col(2) * (kin.H.col(2).transpose() * (kin.P.col(2) + kin.P.col(3)));
    kin.P.col(4) = rand_perp_normal_vec(kin.H.col(3));
    kin.P.col(5) = rand_perp_normal_vec(kin.H.col(4));

    R = rot(rand_normal_vec(), rand_angle());
    T = rand_vec();

  }

  void error() {
    //implement
  }

  // TODO: Fix output for Q and LS to Soln
  void IK_spherical_2_intersecting(const Eigen::Matrix<double, 3, 3>& R_0T, const Eigen::Vector3d& p_0T, const Kin& kin, 
                                  Eigen::Matrix<double, 6, Eigen::Dynamic>& Q, Eigen::Matrix<double, 5, Eigen::Dynamic>& Q_LS) {

    Eigen::Matrix<double, 3, 1> p_16 = (p_0T - R_0T * kin.P.col(6) - kin.P.col(0)); 

    std::vector <double> theta;

    theta.push_back(0);

    bool t3_is_ls = IKS::sp3_run(kin.P.col(3), -kin.P.col(2), kin.H.col(2), p_16.norm(), theta);

    for (unsigned int i = 0; i < theta.size(); i++) {

      double q3 = theta[i];


      std::vector<double> t1, t2;

      t1.push_back(0);
      t2.push_back(0);

      bool t12_is_ls = IKS::sp2_run(p_16,
                              kin.P.col(2) + rot(kin.H.col(2), q3) * kin.P.col(3),
                              -kin.H.col(0),
                              kin.H.col(1),
                              t1, t2);


      for (unsigned int j = 0; j < t1.size(); j++) {
        double q1 = t1[j];
        double q2 = t2[j];

        Eigen::Matrix<double, 3, 3> R_36 = rot(-kin.H.col(2), q3) * rot(-kin.H.col(1), q2) * rot(-kin.H.col(0), q1) * R_0T;

        std::vector<double> t5;

        t5.push_back(0);

        bool q5_is_ls = IKS::sp4_run(kin.H.col(3), kin.H.col(5), kin.H.col(4), 
                                kin.H.col(3).transpose() * R_36 * kin.H.col(5), 
                                t5);


        for (unsigned int k = 0; k < t5.size(); k++) {
          double q5 = t5[k];

          double q4, q6 = 0;

          bool q4_is_ls = IKS::sp1_run(rot(kin.H.col(4), q5) * kin.H.col(5),
                                  R_36*kin.H.col(5),
                                  kin.H.col(4), q4);


          bool q6_is_ls = IKS::sp1_run(rot(-kin.H.col(4), q5) * kin.H.col(3),
                                  R_36.transpose()*kin.H.col(3),
                                  -kin.H.col(6), q6);

          
          Eigen::Matrix<double, 6, 1> q;
          q << q1, q2, q3, q4, q5, q6;
          Q.conservativeResize(6, Q.cols() + 1);
          Q.col(Q.cols() - 1) = q;

          Eigen::Matrix<double, 5, 1> q_ls;
          q_ls << t3_is_ls, t12_is_ls, q5_is_ls, q4_is_ls, q6_is_ls;
          Q_LS.conservativeResize(5, Q_LS.cols() + 1);
          Q_LS.col(Q_LS.cols() - 1) = q_ls;
        }
      }
    }
  }
}
