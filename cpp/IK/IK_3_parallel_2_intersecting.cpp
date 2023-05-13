//---------------------------------------------------------------//
// Name: 3_parallel_2_intersecting.cpp
// Author: Amar Maksumic
// Date: 03/15/2023
// Purpose: Port of the 3_parallel_2_intersecting files
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include "IK_3_parallel_2_intersecting.h"
#include "../read_csv.h"
#include "../subproblems/sp.cpp"

namespace IKS {
  void IK_3_parallel_2_intersecting(const Eigen::Matrix<double, 3, 3> &R_06, const Eigen::Vector3d &p_0T,
                                    const Kin &kin, Soln &soln)
  {

    soln.Q.clear();
    soln.is_LS_vec.clear();
    for (int i = 0; i < 6; i++)
    {
      soln.Q.push_back(std::vector<double>());
      soln.is_LS_vec.push_back(std::vector<bool>());
    }

    Eigen::Matrix<double, 3, 1> p_16 = (p_0T - kin.P.col(0) - R_06 * kin.P.col(6));

    std::vector<double> t1;

    Eigen::Matrix<double, 3, 1> sum;
    sum.row(0) = kin.P.col(1).row(0) + kin.P.col(2).row(0) + kin.P.col(3).row(0) +  kin.P.col(4).row(0);
    sum.row(1) = kin.P.col(1).row(1) + kin.P.col(2).row(1) + kin.P.col(3).row(1) +  kin.P.col(4).row(1);
    sum.row(2) = kin.P.col(1).row(2) + kin.P.col(2).row(2) + kin.P.col(3).row(2) +  kin.P.col(4).row(2);
    double H_twos = kin.H.col(1).transpose() * sum;

    bool t1_is_ls = IKS::sp4_run(kin.H.col(1), p_16, -kin.H.col(0),
                            H_twos, t1);

    for (unsigned int i = 0; i < t1.size(); i++)
    {
      double q1 = t1[i];
      Eigen::Matrix<double, 3, 3> R_01 = rot(kin.H.col(0), q1);
      std::vector<double> t5;

      double d = kin.H.col(1).transpose() * R_01.transpose() * R_06 * kin.H.col(5);

      bool t5_is_ls = IKS::sp4_run(kin.H.col(1), kin.H.col(5), kin.H.col(4),
                              d, t5);

      for (unsigned int j = 0; j < t5.size(); j++)
      {
        double q5 = t5[j];

        double t14;

        bool t14_is_ls = IKS::sp1_run(rot(kin.H.col(4), q5) * kin.H.col(5),
                                R_01.transpose() * R_06 * kin.H.col(5),
                                kin.H.col(1), t14);

        R_01 = rot(kin.H.col(0), q1);
        Eigen::Matrix<double, 3, 3> R_45 = rot(kin.H.col(4), q5);
        Eigen::Matrix<double, 3, 3> R_14 = rot(kin.H.col(1), t14);
        d = (R_01.transpose() * p_16 - kin.P.col(1) - R_14 * (kin.P.col(4) + kin.P.col(5))).norm();

        std::vector<double> t3;

        bool t3_is_ls = IKS::sp3_run(-kin.P.col(3), kin.P.col(2), kin.H.col(1), d, t3);

        for (unsigned int k = 0; k < t3.size(); k++)
        {
          double q3 = t3[k];

          double q2;

          bool q2_is_ls = IKS::sp1_run(kin.P.col(2) + rot(kin.H.col(1), t14) * kin.P.col(3),
                                  R_01.transpose() * p_16 - kin.P.col(1) - R_14 * (kin.P.col(4) + kin.P.col(5)),
                                  kin.H.col(1), q2);

          double q4 = t14 - q2 - q3;

          if (q4 > 0)
            q4 = fmod(q4 + M_PI, 2.0 * M_PI) - M_PI;
          else
            q4 = fmod(q4 - M_PI, 2.0 * M_PI) + M_PI;

          double q6;

          Eigen::Vector3d tr2 =  R_45.transpose() * R_14.transpose() * R_01.transpose() * R_06 * kin.H.col(4);
          bool q6_is_ls = IKS::sp1_run(kin.H.col(4), tr2, kin.H.col(5), q6);

          soln.Q[0].push_back(q1);
          soln.Q[1].push_back(q2);
          soln.Q[2].push_back(q3);
          soln.Q[3].push_back(q4);
          soln.Q[4].push_back(q5);
          soln.Q[5].push_back(q6);
          soln.is_LS_vec[0].push_back(t1_is_ls);
          soln.is_LS_vec[1].push_back(t5_is_ls);
          soln.is_LS_vec[2].push_back(t14_is_ls);
          soln.is_LS_vec[3].push_back(t3_is_ls);
          soln.is_LS_vec[4].push_back(q2_is_ls);
          soln.is_LS_vec[5].push_back(q6_is_ls);
        }
      }
    }
  }
}