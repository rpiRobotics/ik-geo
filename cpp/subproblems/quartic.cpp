#include <iostream>
#include <complex>
#include <vector>
#include <math.h>

const double ZERO_THRESH = 1e-8;

const std::complex<double> i = std::complex<double>(0,0);

std::vector<std::complex<double>> find_roots(double a, double b, double c, double d, double e) {

	double B = b / a, C = c / a, D = d / a, E = e / a;

	double p = -B * B * 0.375 + C;  // double p = -3.0 / 8 * B * B + 1.0;
	double q = pow(B, 3) * 0.125 - B * C * 0.5 + D;
	double r = -pow(B, 4) * 3 / 256 + B * B * C * 0.0625 - B * D * 0.25 + E;

	double tmp1 = 1. / 6 * r * p - 1. / 216 * pow(p, 3) - 1. / 16 * q * q;
	double tmp2 = pow(1. / 3 * r + 1. / 36 * p * p, 3);
	std::complex<double> y1 = p/6 - pow(tmp1-sqrt(pow(tmp1,2)-tmp2+std::complex<double>(0,0)), 1./3) - pow(tmp1+sqrt(pow(tmp1,2)-tmp2+i), 1./3);

	std::complex<double> tmp3 = sqrt(2.*y1 - p);
	std::complex<double> tmp4;
	if (fabs(tmp3.real()) < ZERO_THRESH && fabs(tmp3.imag()) < ZERO_THRESH) tmp4 = 0;
	else tmp4 = -2.0 * q / tmp3;

	std::complex<double> x1 = 0.5 * (tmp3 + sqrt(-p - 2.*y1 + tmp4)) - B * 0.25;
	std::complex<double> x2 = 0.5 * (tmp3 - sqrt(-p - 2.*y1 + tmp4)) - B * 0.25;
	std::complex<double> x3 = 0.5 * (-tmp3 + sqrt(-p - 2.*y1 - tmp4)) - B * 0.25;
	std::complex<double> x4 = 0.5 * (-tmp3 - sqrt(-p - 2.*y1 - tmp4)) - B * 0.25;

	std::vector<std::complex<double>> res;

	res.push_back(x1);
	res.push_back(x2);
	res.push_back(x3);
	res.push_back(x4);

	return res;
}

std::vector<std::complex<double>> quartic_roots(const std::vector<double>& poly) {
	std::vector<std::complex<double>> roots;

	std::complex<double> A = poly[0], B = poly[1], C = poly[2], D = poly[3], E = poly[4];

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

int main() {
	double a, b, c, d, e;
	// std::cin >> a >> b >> c >> d >> e;
	a = 1, b = 2, c = 3, d = 4, e = 5;

	// printf("a: %lf  b: %lf  c: %lf  d: %lf  e: %lf\n", a, b, c, d, e);

	std::vector<std::complex<double>> res;

#if 0
	res = find_roots(a, b, c, d, e);
	printf("find_root:\n");
	for (int i = 0; i < (int)res.size(); i ++ ) {
		printf("x%d: %.3lf + %.3lfi\n", i + 1, res[i].real(), res[i].imag());
	}

	puts("");
#endif

#if 1
	std::vector<double> poly = {a, b, c, d, e};

	printf("poly = [%lf %lf %lf %lf %lf]\n", a, b, c, d, e);

	res = quartic_roots(poly);
	printf("quartic_roots:\n");
	for (int i = 0; i < (int)res.size(); i ++ ) {
		printf("x%d: %.3lf + %.3lfi\n", i + 1, res[i].real(), res[i].imag());
	}
#endif

	return 0;

}