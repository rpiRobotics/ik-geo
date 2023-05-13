#ifndef _HELPER_H_
#define _HELPER_H_

#include <complex>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Polynomials>
#include <vector>


const std::complex<double> i = std::complex<double>(0,0);

void cone_polynomials(const Eigen::Vector3d &p0_i, const Eigen::Vector3d &k_i, const Eigen::Vector3d &p_i, const Eigen::Vector3d &p_i_s, const Eigen::Vector3d &k2, 
					  Eigen::Matrix<double, 1, 2>& P, Eigen::Matrix<double, 1, 3>& R) {
	Eigen::Matrix<double, 3, 1> kiXk2 = k_i.cross(k2);
	Eigen::Matrix<double, 3, 1> kiXkiXk2 = k_i.cross(kiXk2);
	double norm_kiXk2_sq = kiXk2.dot(kiXk2);

	Eigen::Matrix<double, 3, 1> kiXpi = k_i.cross(p_i);
	double norm_kiXpi_sq = kiXpi.dot(kiXpi);

	double delta = k2.dot(p_i_s);
	double alpha = (p0_i.transpose() * kiXkiXk2 / norm_kiXk2_sq)(0, 0);
	double beta = (p0_i.transpose() * kiXk2 / norm_kiXk2_sq)(0, 0);

	double P_const = norm_kiXpi_sq + p_i_s.dot(p_i_s) + 2*alpha*delta;
	P << -2*alpha, P_const;

	R << -1, 2*delta, -pow(delta, 2);
	R(0, 2) = R(0, 2) + norm_kiXpi_sq*norm_kiXk2_sq;
	R = pow(2*beta, 2) * R;
	return;
}

Eigen::Matrix<double, 1, 3> convolution_2(Eigen::Matrix<double, 1, 2> &v1, Eigen::Matrix<double, 1, 2> &v2) {
	Eigen::Matrix<double, 1, 3> res;
	res << v1(0, 0)*v2(0, 0), v1(0, 0)*v2(0, 1)+v1(0, 1)*v2(0, 0), v1(0, 1)*v2(0, 1);
	return res;
}

Eigen::Matrix<double, 1, 5> convolution_3(Eigen::Matrix<double, 1, 3> &v1, Eigen::Matrix<double, 1, 3> &v2) {
	Eigen::Matrix<double, 1, 5> res;
	res << v1(0, 0)*v2(0, 0), v1(0, 1)*v2(0, 0)+v1(0, 0)*v2(0, 1), v1(0, 0)*v2(0, 2)+v1(0, 1)*v2(0, 1)+v1(0, 2)*v2(0, 0), 
		   v1(0, 1)*v2(0, 2)+v1(0, 2)*v2(0, 1), v1(0, 2)*v2(0, 2);
	return res;
}

std::vector<std::complex<double>> quartic_roots(const Eigen::Matrix<double, 1, 5> &poly) {
	std::vector<std::complex<double>> roots;

	double A = poly(0, 0), B = poly(0, 1), C = poly(0, 2), D = poly(0, 3), E = poly(0, 4);

	std::complex<double> alpha = -0.375*B*B/(A*A) + C/A;
	std::complex<double> beta = 0.125*B*B*B/(A*A*A) - 0.5*B*C/(A*A) + D/A;
	std::complex<double> gamma = -B*B*B*B*3./(A*A*A*A*256.) + C*B*B/(A*A*A*16.) - B*D/(A*A*4.) + E/A;

	if (fabs(beta.real()) < 1e-12 && fabs(beta.imag()) < 1e-12) {
		std::complex<double> tmp = sqrt(alpha*alpha - gamma*4. + i);
		roots.push_back(-B/(A*4.) + sqrt((-alpha + tmp)/2. + i));
		roots.push_back(-B/(A*4.) - sqrt((-alpha + tmp)/2. + i));
		roots.push_back(-B/(A*4.) + sqrt((-alpha - tmp)/2. + i));
		roots.push_back(-B/(A*4.) - sqrt((-alpha - tmp)/2. + i));
		return roots;
	}

	std::complex<double> P = -alpha*alpha/12. - gamma;
	std::complex<double> Q = -alpha*alpha*alpha/108. + alpha*gamma/3. - beta*beta*0.125;
	std::complex<double> R = -Q*0.5 + sqrt(Q*Q*0.25 + P*P*P/27. + i);
	std::complex<double> U = pow(R, 1./3);

	std::complex<double> y;
	if (fabs(U.real()) < 1e-12 && fabs(U.imag()) < 1e-12) {
		y = -alpha*5./6. - pow(Q, 1./3);
	}
	else {
		y = -alpha*5./6. + U - P/(3.*U);
	}

	std::complex<double> W = sqrt(alpha + 2.*y + i);

	roots.push_back(-B/(A*4.) + (W + sqrt(-(alpha*3. + 2.*y + beta*2./W)))/2.);
	roots.push_back(-B/(A*4.) + (W - sqrt(-(alpha*3. + 2.*y + beta*2./W)))/2.);
	roots.push_back(-B/(A*4.) - (W + sqrt(-(alpha*3. + 2.*y - beta*2./W)))/2.);
	roots.push_back(-B/(A*4.) - (W - sqrt(-(alpha*3. + 2.*y - beta*2./W)))/2.);

	return roots;
}

void find_quartic_roots(Eigen::Matrix<double, 5, 1>& coeffs, 
                        Eigen::Matrix<std::complex<double>, 4, 1>& roots) {
  /* Find the roots of a quartic polynomial */
  std::complex<double> a = coeffs.coeffRef(0,0);
  std::complex<double> b = coeffs.coeffRef(1,0);
  std::complex<double> c = coeffs.coeffRef(2,0);
  std::complex<double> d = coeffs.coeffRef(3,0);
  std::complex<double> e = coeffs.coeffRef(4,0);

  std::complex<double> p1 = 2.0*c*c*c - 9.0*b*c*d + 27.0*a*d*d + 27.0*b*b*e - 72.0*a*c*e;
  std::complex<double> q1 = c*c - 3.0*b*d + 12.0*a*e;
  std::complex<double> p2 = p1 + sqrt(-4.0*q1*q1*q1 + p1*p1);
  std::complex<double> q2 = cbrt(p2.real() / 2.0);
  std::complex<double> p3 = q1 / (3.0*a*q2) + q2 / (3.0*a);
  std::complex<double> p4 = sqrt((b*b) / (4.0*a*a) - (2.0*c) / (3.0*a) + p3);
  std::complex<double> p5 = (b*b) / (2.0*a*a) - (4.0*c) / (3.0*a) - p3;
  std::complex<double> p6 = (-(b*b*b) / (a*a*a) + (4.0*b*c) / (a*a) - (8.0*d) / a) / (4.0*p4);

  roots(0,0) = -b / (4.0*a) - p4 / 2.0 - sqrt(p5 - p6) / 2.0;
  roots(1,0) = -b / (4.0*a) - p4 / 2.0 + sqrt(p5 - p6) / 2.0;
  roots(2,0) = -b / (4.0*a) + p4 / 2.0 - sqrt(p5 + p6) / 2.0;
  roots(3,0) = -b / (4.0*a) + p4 / 2.0 + sqrt(p5 + p6) / 2.0;
}


void solve_2_ellipse_numeric(Eigen::Vector2d& xm1, Eigen::Matrix<double, 2, 2>& xn1, 
                             Eigen::Vector2d& xm2, Eigen::Matrix<double, 2, 2>& xn2,
                             Eigen::Matrix<double, 4, 1>& xi_1, Eigen::Matrix<double, 4, 1>& xi_2) {
  /* solve for intersection of 2 ellipses defined by

   xm1'*xm1 + xi'*xn1'*xn1*xi  + xm1'*xn1*xi == 1
   xm2'*xm2 + xi'*xn2'*xn2*xi  + xm2'*xn2*xi == 1
   Where xi = [xi_1; xi_2] */
  
  Eigen::Matrix<double, 2, 2> A_1 = xn1.transpose() * xn1;
  double a = A_1.coeffRef(0,0);
  double b = 2*A_1.coeffRef(1,0);
  double c = A_1.coeffRef(1, 1);
  Eigen::Matrix<double, 1, 2> B_1 = 2*xm1.transpose() * xn1;
  double d = B_1.coeffRef(0,0);
  double e = B_1.coeffRef(0,1);
  double f = xm1.transpose() * xm1 - 1;

  Eigen::Matrix<double, 2, 2> A_2 = xn2.transpose() * xn2;
  double a1 = A_2.coeffRef(0,0);
  double b1 = 2*A_2.coeffRef(1,0);
  double c1 = A_2.coeffRef(1, 1);
  Eigen::Matrix<double, 1, 2> B_2 = 2*xm2.transpose() * xn2;
  double d1 = B_2.coeffRef(0,0);
  double e1 = B_2.coeffRef(0,1);
  double fq = xm2.transpose() * xm2 - 1;

  double z0 = f*a*d1*d1+a*a*fq*fq-d*a*d1*fq+a1*a1*f*f-2*a*fq*a1*f-d*d1*a1*f+a1*d*d*fq;

  double z1 = e1*d*d*a1-fq*d1*a*b-2*a*fq*a1*e-f*a1*b1*d+2*d1*b1*a*f+2*e1*fq*a*a+d1*d1*a*e-e1*d1*a*d-2*a*e1*a1*f-f*a1*d1*b+2*f*e*a1*a1-fq*b1*a*d-e*a1*d1*d+2*fq*b*a1*d;

  double z2 = e1*e1*a*a+2*c1*fq*a*a-e*a1*d1*b+fq*a1*b*b-e*a1*b1*d-fq*b1*a*b-2*a*e1*a1*e+2*d1*b1*a*e-c1*d1*a*d-2*a*c1*a1*f+b1*b1*a*f+2*e1*b*a1*d+e*e*a1*a1-c*a1*d1*d-e1*b1*a*d+2*f*c*a1*a1-f*a1*b1*b+c1*d*d*a1+d1*d1*a*c-e1*d1*a*b-2*a*fq*a1*c;

  double z3 = -2*a*a1*c*e1+e1*a1*b*b+2*c1*b*a1*d-c*a1*b1*d+b1*b1*a*e-e1*b1*a*b-2*a*c1*a1*e-e*a1*b1*b-c1*b1*a*d+2*e1*c1*a*a+2*e*c*a1*a1-c*a1*d1*b+2*d1*b1*a*c-c1*d1*a*b;

  double z4 = a*a*c1*c1-2*a*c1*a1*c+a1*a1*c*c-b*a*b1*c1-b*b1*a1*c+b*b*a1*c1+c*a*b1*b1;

  Eigen::Matrix<double, 5, 1> z(z0, z1, z2, z3, z4);
  Eigen::Matrix<std::complex<double>, 4, 1> roots;
  find_quartic_roots(z, roots);

  for (int i = 0; i < 4; i++) {
    double y_r = roots.coeffRef(i, 0).real();
    double y_sq = y_r * y_r;
    xi_2(i, 0) = y_r;

    double x_r = -(a*fq+a*c1*y_sq-a1*c*y_sq+a*e1*y_r-a1*e*y_r-a1*f)/(a*b1*y_r+a*d1-a1*b*y_r-a1*d);
    xi_1(i,0) = x_r;
  }
}


#endif
