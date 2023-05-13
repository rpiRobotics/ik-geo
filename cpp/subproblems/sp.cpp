//---------------------------------------------------------------//
// Name: sp_1.cpp
// Author: Runbin Chen and Amar Maksumic
// Purpose: Port of the subproblem files functionality
//---------------------------------------------------------------//
#include "sp.h"
#include "../rand_cpp.h"

namespace IKS {
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
	
		const std::complex<double> i = std::complex<double>(0,0);
		
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

		Eigen::Matrix<double, 5, 1> z;
		z << z0, z1, z2, z3, z4;
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

	// ===== SUBPROBLEMS ===== //

	bool sp1_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
							const Eigen::Vector3d& k, 
							double& theta) {

		Eigen::Matrix<double, 3, 1> KxP = k.cross(p1);
		Eigen::Matrix<double, 3, 2> A;
		A << KxP, -k.cross(KxP);

		Eigen::Vector2d x = A.transpose() * p2;

		theta = atan2(x(0), x(1));

		return fabs(p1.norm() - p2.norm()) > ZERO_THRESH || fabs(k.dot(p1) - k.dot(p2)) > ZERO_THRESH;
	}

	bool sp2_run(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
							const Eigen::Vector3d& k1, const Eigen::Vector3d& k2, 
							std::vector<double>& theta1, std::vector<double>& theta2) {

		Eigen::Vector3d p_1 = p1/p1.norm();
		Eigen::Vector3d p_2 = p2/p2.norm();

		Eigen::Matrix<double, 3, 1> KxP1 = k1.cross(p_1);
		Eigen::Matrix<double, 3, 1> KxP2 = k2.cross(p_2);

		Eigen::Matrix<double, 3, 2> A_1, A_2; 
		A_1 << KxP1, -k1.cross(KxP1);
		A_2 << KxP2, -k2.cross(KxP2);

		double radius_1_sq = KxP1.dot(KxP1);
		double radius_2_sq = KxP2.dot(KxP2);

		double k1_d_p1 = k1.dot(p_1);
		double k2_d_p2 = k2.dot(p_2);
		double k1_d_k2 = k1.dot(k2);

		double ls_frac = 1/(1-(k1_d_k2*k1_d_k2));
		double alpha_1 = ls_frac * (k1_d_p1 - k1_d_k2 * k2_d_p2);
		double alpha_2 = ls_frac * (k2_d_p2 - k1_d_k2 * k1_d_p1);

		Eigen::Matrix<double, 2, 1> x_ls_1 = (alpha_2 * A_1.transpose() * (k2)) / radius_1_sq;
		Eigen::Matrix<double, 2, 1> x_ls_2 = (alpha_1 * A_2.transpose() * (k1)) / radius_2_sq;
		Eigen::Matrix<double, 4, 1> x_ls;
		x_ls << x_ls_1, x_ls_2; 

		Eigen::Matrix<double, 3, 1> n_sym = k1.cross(k2);
		Eigen::Matrix<double, 2, 3> pinv_A1, pinv_A2;
		pinv_A1 = A_1.transpose() / radius_1_sq;
		pinv_A2 = A_2.transpose() / radius_2_sq;

		Eigen::Matrix<double, 4, 1> A_perp_tilde;
		Eigen::Matrix<double, 4, 3> temp;
		temp << pinv_A1, pinv_A2;
		A_perp_tilde = temp * n_sym;

		if (x_ls.block<2, 1>(0,0).norm() < 1) {
			double xi = sqrt(1 - pow(x_ls.block<2, 1>(0,0).norm(), 2)) / A_perp_tilde.block<2, 1>(0, 0).norm();
			Eigen::Matrix<double, 4, 1> sc_1 = x_ls + xi*A_perp_tilde;
			Eigen::Matrix<double, 4, 1> sc_2 = x_ls - xi*A_perp_tilde;

			theta1[0] = (atan2(sc_1(0, 0), sc_1(1, 0)));
			theta1[0] = (atan2(sc_2(0, 0), sc_2(1, 0)));
			theta2.push_back(atan2(sc_1(0, 0), sc_1(1, 0)));
			theta2.push_back(atan2(sc_2(0, 0), sc_2(1, 0)));
			return false;
		} else {
			theta1[0] = (atan2(x_ls(0, 0), x_ls(1, 0)));
			theta2[0] = (atan2(x_ls(2, 0), x_ls(3, 0)));
			return true;
		}
	}

	void sp2E_run(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
								const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, 
								double &theta1, double &theta2) {

		Eigen::Matrix<double, 3, 1> KxP1 = k1.cross(p1);
		Eigen::Matrix<double, 3, 1> KxP2 = k2.cross(p2);

		Eigen::Matrix<double, 3, 2> A_1, A_2;
		A_1 << KxP1, -k1.cross(KxP1);
		A_2 << KxP2, -k2.cross(KxP2);

		Eigen::Matrix<double, 3, 4> A;
		A << A_1, -A_2;

		Eigen::Vector3d p = -k1*k1.dot(p1) + k2*k2.dot(p2) - p0;

		double radius_1_sp = KxP1.dot(KxP1);
		double radius_2_sp = KxP2.dot(KxP2);

		double alpha = radius_1_sp / (radius_1_sp + radius_2_sp);
		double beta = radius_2_sp / (radius_1_sp + radius_2_sp);
		Eigen::Matrix<double, 3, 3> M_inv, AAT_inv;
		M_inv = Eigen::Matrix3d::Identity(3, 3) + k1*k1.transpose()*(alpha/(1-alpha));
		AAT_inv = 1/(radius_1_sp+radius_2_sp)*(M_inv + M_inv*k2*k2.transpose()*M_inv*beta/(1.0-(k2.transpose()*M_inv*k2*beta)(0, 0)));
		Eigen::Matrix<double, 4, 1> x_ls = A.transpose() * AAT_inv * p;

		Eigen::Matrix<double, 3, 1> n_sym = k1.cross(k2);
		Eigen::Matrix<double, 2, 3> pinv_A1, pinv_A2;
		pinv_A1 = A_1.transpose() / radius_1_sp;
		pinv_A2 = A_2.transpose() / radius_2_sp;
		Eigen::Matrix<double, 4, 1> A_perp_tilde;
		Eigen::Matrix<double, 4, 3> temp;
		temp << pinv_A1, 
		pinv_A2;
		A_perp_tilde = temp * n_sym;

		double num = (pow(x_ls.block<2, 1>(2, 0).norm(), 2)-1)*pow(A_perp_tilde.block<2, 1>(0, 0).norm(), 2) 
		- (pow(x_ls.block<2, 1>(0, 0).norm(), 2)-1)*pow(A_perp_tilde.block<2, 1>(2, 0).norm(), 2);
		double den = 2*(x_ls.block<2, 1>(0, 0).transpose()*A_perp_tilde.block<2, 1>(0, 0)*pow(A_perp_tilde.block<2, 1>(2, 0).norm(), 2) \
			- x_ls.block<2, 1>(2, 0).transpose()*A_perp_tilde.block<2, 1>(2, 0)*pow(A_perp_tilde.block<2, 1>(0, 0).norm(), 2))(0, 0);

		double xi = num / den;

		Eigen::Matrix<double, 4, 1> sc = x_ls + xi*A_perp_tilde;

		theta1 = atan2(sc(0, 0), sc(1, 0));
		theta2 = atan2(sc(2, 0), sc(3, 0));
	}

	bool sp3_run(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
							const Eigen::Vector3d &k, 
							const double &d, 
							std::vector<double> &theta) {

		Eigen::Matrix<double, 3, 1> KxP = k.cross(p1);

		Eigen::Matrix<double, 3, 2> A_1;
		A_1 << KxP, -k.cross(KxP);
		Eigen::Matrix<double, 1, 2> A;
		A = -2 * p2.transpose() * A_1;
		double norm_A_sq = A.dot(A);
		double norm_A = sqrt(norm_A_sq);

		double b = pow(d, 2) - pow((p2-k*k.transpose()*p1).norm(), 2) - pow(KxP.norm(), 2);

		Eigen::Matrix<double, 2, 1> x_ls = A_1.transpose() * (-2*p2*b/norm_A_sq);

		theta.clear();
		if (x_ls.dot(x_ls) > 1) {
			theta.push_back(atan2(x_ls(0, 0), x_ls(1, 0)));
			return true;
		}

		double xi = sqrt(1-pow(b, 2)/norm_A_sq);

		Eigen::Matrix<double, 2, 1> A_perp_tilde, A_perp;
		A_perp_tilde << A(0, 1), -A(0, 0);
		A_perp = A_perp_tilde / norm_A;

		Eigen::Matrix<double, 2, 1> sc_1, sc_2;
		sc_1 = x_ls + xi*A_perp;
		sc_2 = x_ls - xi*A_perp;

		theta.push_back(atan2(sc_1(0, 0), sc_1(1, 0)));
		theta.push_back(atan2(sc_2(0, 0), sc_2(1, 0)));

		return false;
	}

	bool sp4_run(const Eigen::Vector3d& p, 
							const Eigen::Vector3d& k, 
							const Eigen::Vector3d& h, 
							const double& d, 
							std::vector<double>& theta) {

		Eigen::Matrix<double, 3, 1> A_11 = k.cross(p);
		Eigen::Matrix<double, 3, 2> A_1;
		A_1 << A_11, -k.cross(A_11);

		Eigen::Matrix<double, 1, 2> A = h.transpose() * A_1;

		double b = d - (h.transpose() * (k * (k.transpose() * p)));

		double norm_A_2 = A.dot(A);

		Eigen::Matrix<double, 2, 1> x_ls = A_1.transpose() * (h * b);

		if (norm_A_2 > b*b) {
			double sqrt_2_b = norm_A_2 - b * b;
			double xi = sqrt(sqrt_2_b);

			Eigen::Matrix<double, 2, 1> A_perp_tilde;
			A_perp_tilde << A(1), -A(0);

			Eigen::Matrix<double, 2, 1> sc_1 = x_ls + xi*A_perp_tilde;
			Eigen::Matrix<double, 2, 1> sc_2 = x_ls - xi*A_perp_tilde;

			double theta_1 = atan2(sc_1(0, 0), sc_1(1, 0));
			double theta_2 = atan2(sc_2(0, 0), sc_2(1, 0));
			theta[0] = (theta_1);
			theta.push_back(theta_2);
			return false;
		} else {
			theta[0] = (atan2(x_ls(0, 0), x_ls(1, 0)));
			return true;
		}
	}

	void sp5_run(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, 
						const Eigen::Vector3d &k1, const Eigen::Vector3d &k2, const Eigen::Vector3d &k3, 
						std::vector<double> &theta1, std::vector<double> &theta2, std::vector<double> &theta3) {

			theta1 = std::vector<double>(0);
			theta2 = std::vector<double>(0);
			theta3 = std::vector<double>(0);
			int i_soln = 0;

			Eigen::Matrix<double, 3, 1> p1_s = p0 + k1*k1.transpose()*p1;
			Eigen::Matrix<double, 3, 1> p3_s = p2 + k3*k3.transpose()*p3;

			double delta1 = k2.dot(p1_s);
			double delta3 = k2.dot(p3_s);

			Eigen::Matrix<double, 1, 2> P_1, P_3;
			Eigen::Matrix<double, 1, 3> R_1, R_3;
			cone_polynomials(p0, k1, p1, p1_s, k2, P_1, R_1);
			cone_polynomials(p2, k3, p3, p3_s, k2, P_3, R_3);
			
			Eigen::Matrix<double, 1, 2> P_13 = P_1 - P_3;
			Eigen::Matrix<double, 1, 3> P_13_sq = convolution_2(P_13, P_13);

			Eigen::Matrix<double, 1, 3> RHS = R_3 - R_1 - P_13_sq;

			Eigen::Matrix<double, 1, 5> EQN = convolution_3(RHS, RHS) - 4*convolution_3(P_13_sq, R_1);

			std::vector<std::complex<double>> all_roots = quartic_roots(EQN);
			std::vector<double> H_vec;
			for (int i = 0; i < (int)all_roots.size(); i ++ ) {
				if (fabs(all_roots[i].imag()) < 1e-6)
					H_vec.push_back(all_roots[i].real());
			}

			Eigen::Matrix<double, 3, 1> KxP1 = k1.cross(p1);
			Eigen::Matrix<double, 3, 1> KxP3 = k3.cross(p3);
			Eigen::Matrix<double, 3, 2> A_1;
			A_1 << KxP1, -k1.cross(KxP1);
			Eigen::Matrix<double, 3, 2> A_3;
			A_3 << KxP3, -k3.cross(KxP3);

			std::vector<std::vector<int>> signs(2);
			signs[0] = {1, 1, -1, -1};
			signs[1] = {1, -1, 1, -1};
			Eigen::Matrix<double, 2, 2> J;
			J << 0, 1, 
			-1, 0;

			for (int i_H = 0; i_H < (int)H_vec.size(); i_H ++ ) {
				double H = H_vec[i_H];

				Eigen::Matrix<double, 2, 1> const_1 = A_1.transpose() * k2 * (H - delta1);
				Eigen::Matrix<double, 2, 1> const_3 = A_3.transpose() * k2 * (H - delta3);
				Eigen::Matrix<double, 2, 1> pm_1 = J*A_1.transpose()*k2*sqrt(pow((A_1.transpose()*k2).norm(), 2) - pow(H-delta1, 2));
				Eigen::Matrix<double, 2, 1> pm_3 = J*A_3.transpose()*k2*sqrt(pow((A_3.transpose()*k2).norm(), 2) - pow(H-delta3, 2));
				
				for (int i_sign = 0; i_sign < 4; i_sign ++ ) {
					int sign_1 = signs[0][i_sign];
					int sign_3 = signs[1][i_sign];

					Eigen::Matrix<double, 2, 1> sc1 = const_1 + (double)sign_1 * pm_1;
					sc1 = sc1 / pow((A_1.transpose()*k2).norm(), 2);

					Eigen::Matrix<double, 2, 1> sc3 = const_3 + (double)sign_3 * pm_3;
					sc3 = sc3 / pow((A_3.transpose()*k2).norm(), 2);

					Eigen::Matrix<double, 3, 1> v1 = A_1*sc1 + p1_s;
					Eigen::Matrix<double, 3, 1> v3 = A_3*sc3 + p3_s;
					
					if (fabs((v1-H*k2).norm() - (v3-H*k2).norm()) < 1e-6) {
						i_soln ++ ;
						theta1.push_back(atan2(sc1(0, 0), sc1(1, 0)));
					// theta2[i_soln] = subproblem.sp_1(v3, v1, k2);
						double theta;
						sp1_run(v3, v1, k2, theta);
						theta2.push_back(theta);
						theta3.push_back(atan2(sc3(0, 0), sc3(1, 0)));
					}
				}

			}
	}

	void sp6_run(Eigen::Matrix<double, 3, 4>& p, 
							Eigen::Matrix<double, 3, 4>& k, 
							Eigen::Matrix<double, 3, 4>& h, 
							double& d1, double& d2,
							std::vector<double> &theta1, std::vector<double> &theta2) {

		Eigen::Vector3d k1Xp1 = k.col(0).cross(p.col(0));
		Eigen::Vector3d k2Xp2 = k.col(1).cross(p.col(1));
		Eigen::Vector3d k3Xp3 = k.col(2).cross(p.col(2));
		Eigen::Vector3d k4Xp4 = k.col(3).cross(p.col(3));

		Eigen::Matrix<double, 3, 2> A_1;
		A_1 << k1Xp1, -k.col(0).cross(k1Xp1);
		Eigen::Matrix<double, 3, 2> A_2;
		A_2 << k2Xp2, -k.col(1).cross(k2Xp2);
		Eigen::Matrix<double, 3, 2> A_3;
		A_3 << k3Xp3, -k.col(2).cross(k3Xp3);
		Eigen::Matrix<double, 3, 2> A_4;
		A_4 << k4Xp4, -k.col(3).cross(k4Xp4);

		Eigen::Matrix<double, 2, 4> A;
		A << (h.col(0).transpose() * A_1), (h.col(1).transpose() * A_2),
			(h.col(2).transpose() * A_3), (h.col(3).transpose() * A_4);

		Eigen::Matrix<double, 4, 1> x_min;
		Eigen::Matrix<double, 2, 1> den;
		den << (d1 - h.col(0).transpose() * k.col(0) * k.col(0).transpose() * p.col(0) - h.col(1).transpose() * k.col(1) * k.col(1).transpose() * p.col(1)),
				(d2 - h.col(2).transpose() * k.col(2) * k.col(2).transpose() * p.col(2) - h.col(3).transpose() * k.col(3) * k.col(3).transpose() * p.col(3));
		x_min = A.colPivHouseholderQr().solve(den);

		Eigen::CompleteOrthogonalDecomposition<
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> cod;
		cod.compute(A);
		unsigned rk = cod.rank();
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P =
				cod.colsPermutation();
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V =
				cod.matrixZ().transpose();
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x_null =
				P * V.block(0, rk, V.rows(), V.cols() - rk);

		Eigen::Matrix<double, 4, 1> x_null_1 = x_null.col(0);
		Eigen::Matrix<double, 4, 1> x_null_2 = x_null.col(1);

		Eigen::Matrix<double, 4, 1> xi_1;
		Eigen::Matrix<double, 4, 1> xi_2;

		Eigen::Matrix<double, 2, 1> x_min_1 = x_min.block<2, 1>(0,0);
		Eigen::Matrix<double, 2, 1> x_min_2 = x_min.block<2, 1>(2,0);
		// std::cout << x_null.rows() << " " << x_null.cols() << std::endl;
		Eigen::Matrix<double, 2, 2> x_n_1 = x_null.block<2, 2>(0,0);
		Eigen::Matrix<double, 2, 2> x_n_2 = x_null.block<2, 2>(2,0);

		solve_2_ellipse_numeric(x_min_1, x_n_1, x_min_2, x_n_2, xi_1, xi_2);

		theta1.clear();
		theta2.clear();

		for (int i = 0; i < 4; i++) {
			Eigen::Matrix<double, 4, 1> x = x_min + x_null_1 * xi_1(i, 0) + x_null_2 * xi_2(i, 0);
			theta1.push_back(atan2(x(0, 0), x(1, 0)));
			theta2.push_back(atan2(x(2, 0), x(3, 0)));
		}
	}
}