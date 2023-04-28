//---------------------------------------------------------------//
// Name: 3_parallel_2_intersecting.cpp
// Author: Amar Maksumic
// Date: 03/15/2023
// Purpose: Port of the 3_parallel_2_intersecting files
//---------------------------------------------------------------//

#pragma GCC optimize(3)

#include <eigen3/Eigen/Dense>
#include <vector>
#include "IK_3_parallel_2_intersecting.h"
#include "../rand_cpp.h"
#include "../+subproblem_setups/sp_1.cpp"
#include "../+subproblem_setups/sp_2.cpp"
#include "../+subproblem_setups/sp_3.cpp"
#include "../+subproblem_setups/sp_4.cpp"

// TODO: Fix output for Q and LS to Soln
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

  bool t1_is_ls = sp4_run(kin.H.col(1), p_16, -kin.H.col(0),
                          H_twos, t1);

  for (unsigned int i = 0; i < t1.size(); i++)
  {
    double q1 = t1[i];
    Eigen::Matrix<double, 3, 3> R_01 = rot(kin.H.col(0), q1);
    std::vector<double> t5;

    double d = kin.H.col(1).transpose() * R_01.transpose() * R_06 * kin.H.col(5);

    bool t5_is_ls = sp4_run(kin.H.col(1), kin.H.col(5), kin.H.col(4),
                            d, t5);

    for (unsigned int j = 0; j < t5.size(); j++)
    {
      double q5 = t5[j];

      double t14;

      bool t14_is_ls = sp1_run(rot(kin.H.col(4), q5) * kin.H.col(5),
                               R_01.transpose() * R_06 * kin.H.col(5),
                               kin.H.col(1), t14);

      R_01 = rot(kin.H.col(0), q1);
      Eigen::Matrix<double, 3, 3> R_45 = rot(kin.H.col(4), q5);
      Eigen::Matrix<double, 3, 3> R_14 = rot(kin.H.col(1), t14);
      d = (R_01.transpose() * p_16 - kin.P.col(1) - R_14 * (kin.P.col(4) + kin.P.col(5))).norm();

      std::vector<double> t3;

      bool t3_is_ls = sp_3(-kin.P.col(3), kin.P.col(2), kin.H.col(1), d, t3);

      for (unsigned int k = 0; k < t3.size(); k++)
      {
        double q3 = t3[k];

        double q2;

        bool q2_is_ls = sp1_run(kin.P.col(2) + rot(kin.H.col(1), t14) * kin.P.col(3),
                                R_01.transpose() * p_16 - kin.P.col(1) - R_14 * (kin.P.col(4) + kin.P.col(5)),
                                kin.H.col(1), q2);

        double q4 = t14 - q2 - q3;

        if (q4 > 0)
          q4 = fmod(q4 + M_PI, 2.0 * M_PI) - M_PI;
        else
          q4 = fmod(q4 - M_PI, 2.0 * M_PI) + M_PI;

        double q6;

        Eigen::Vector3d tr2 =  R_45.transpose() * R_14.transpose() * R_01.transpose() * R_06 * kin.H.col(4);
        bool q6_is_ls = sp1_run(kin.H.col(4), tr2, kin.H.col(5), q6);

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

int main(int argc, char **argv)
{

  std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("dat.csv");
  if (data.size() != 57)
  {
    std::cerr << "Invalid input data for IK3P2I. \n";
    return 0;
  }

  double time_avg = 0;

  for (int i = 0; i < (int)data[0].second.size(); i++)
  {
    Kin kin;
    Eigen::Matrix<double, 3, 3> R_06;
    Eigen::Vector3d p_0T;
    std::vector<double> Q;
    Soln soln;
    kin.H << data[0].second[i], data[1].second[i], data[2].second[i], data[3].second[i], data[4].second[i], data[5].second[i],
             data[6].second[i], data[7].second[i], data[8].second[i], data[9].second[i], data[10].second[i], data[11].second[i],
             data[12].second[i], data[13].second[i], data[14].second[i], data[15].second[i], data[16].second[i], data[17].second[i];
    kin.P << data[18].second[i], data[19].second[i], data[20].second[i], data[21].second[i], data[22].second[i], data[23].second[i], data[24].second[i],
             data[25].second[i], data[26].second[i], data[27].second[i], data[28].second[i], data[29].second[i], data[30].second[i], data[31].second[i],
             data[32].second[i], data[33].second[i], data[34].second[i], data[35].second[i], data[36].second[i], data[37].second[i], data[38].second[i];
    R_06 << data[39].second[i], data[40].second[i], data[41].second[i], 
            data[42].second[i], data[43].second[i], data[44].second[i], 
            data[45].second[i], data[46].second[i], data[47].second[i];
    p_0T << data[48].second[i], data[49].second[i], data[50].second[i];
    // Q << data[51].second[i], data[52].second[i], data[53].second[i], data[54].second[i], data[55].second[i], data[56].second[i];
    auto start = std::chrono::steady_clock::now();

    IK_3_parallel_2_intersecting(R_06, p_0T, kin, soln);
    auto end = std::chrono::steady_clock::now();

    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  }

  time_avg /= (int)data[0].second.size();

  std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl;

  return 0;
}