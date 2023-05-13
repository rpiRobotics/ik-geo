#ifndef _SP_H_
#define _SP_H_

#include <math.h>
#include <vector>
#include <complex>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Polynomials>


namespace IKS {

  #ifndef ZERO_THRESH
  #define ZERO_THRESH 1e-8
  #endif

  bool sp1_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
              const Eigen::Vector3d& k, 
              double& theta);


  bool sp2_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
              const Eigen::Vector3d& k1, const Eigen::Vector3d& k2, 
              std::vector<double>& theta1, std::vector<double>& theta2);

  void sp2E_run(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
                const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, 
                double &theta1, double &theta2);

  bool sp3_run(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
              const Eigen::Vector3d &k, 
              const double &d, 
              std::vector<double> &theta);

  bool sp4_run(const Eigen::Vector3d& p, 
              const Eigen::Vector3d& k, 
              const Eigen::Vector3d& h, 
              const double& d, 
              std::vector<double>& theta);

  void sp5_run(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, 
            const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, const Eigen::Vector3d &k3, 
            std::vector<double> &theta1, std::vector<double> &theta2, std::vector<double> &theta3);

  void sp6_run(Eigen::Matrix<double, 3, 4>& p, 
              Eigen::Matrix<double, 3, 4>& k, 
              Eigen::Matrix<double, 3, 4>& h, 
              double& d1, double& d2, 
              std::vector<double> &theta1, std::vector<double> &theta2);
}

#endif