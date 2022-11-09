use {
    super::solutionset::{ SolutionSet2, SolutionSet4 },

    nalgebra::{
        Vector2, Vector3, Vector4, Vector5,
        Matrix, Matrix3, Matrix4, Matrix3x2, Matrix3x4, Matrix4x1, Matrix4x3,
        ArrayStorage, U1, U8,
        Complex, DVector, Matrix2,
    }
};

pub type Vector8<T> = Matrix<T, U8, U1, ArrayStorage<T, 8, 1>>;

/**
Solves for `theta` where `rot(k, theta) * p1 = p2` if possible.
If not, minimizes `|| rot(k, theta) * p1 - p2 ||`.
Also returns a boolean of whether or not `theta` is a least-squares solution.
*/
pub fn subproblem1(p1: &Vector3<f64>, p2: &Vector3<f64>, k: &Vector3<f64>) -> (f64, bool) {
    let kxp = k.cross(p1);
    let a = Matrix3x2::from_columns(&[kxp, -k.cross(&kxp)]);
    let x = a.transpose() * p2;

    let theta = x[0].atan2(x[1]);
    let is_ls = (p1.norm() - p2.norm()).abs() > 1e-8 || (k.dot(p1) - k.dot(p2)).abs() > 1e-8;

    (theta, is_ls)
}

// NOTE: Maybe we should be using types like SolutionSet2<(f64, f64)> rather than (SolutionSet2<f64>, SolutionSet2<f64>)

/**
Solves for `theta1` and `theta2` where `rot(k1, theta1) * p1 = rot(k2, theta2) * p2` if possible.
If not, minimizes `|| rot(k1, theta1) * p1 - rot(k2, theta2) * p2 ||`.
Also returns a boolean of whether or not `{ theta1, theta2 }` is a least-squares solution.
There may be 1 or 2 solutions for `theta1` and `theta2`.
*/
pub fn subproblem2(p1: &Vector3<f64>, p2: &Vector3<f64>, k1: &Vector3<f64>, k2: &Vector3<f64>) -> (SolutionSet2<f64>, SolutionSet2<f64>, bool) {
    let is_ls = (p1.norm() - p2.norm()).abs() >= 1e-8;

    let p1 = p1.normalize();
    let p2 = p2.normalize();

    let kxp1 = k1.cross(&p1);
    let kxp2 = k2.cross(&p2);
    let a1  = Matrix3x2::from_columns(&[kxp1, -k1.cross(&kxp1)]);
    let a2  = Matrix3x2::from_columns(&[kxp2, -k2.cross(&kxp2)]);

    let radius_1_sq = kxp1.dot(&kxp1);
    let radius_2_sq = kxp2.dot(&kxp2);

    let k1_d_p1 = k1.dot(&p1);
    let k2_d_p2 = k2.dot(&p2);
    let k1_d_k2 = k1.dot(&k2);
    let ls_frac = 1.0 / (1.0 - k1_d_k2 * k1_d_k2);
    let alpha_1 = ls_frac * (k1_d_p1 - k1_d_k2 * k2_d_p2);
    let alpha_2 = ls_frac * (k2_d_p2 - k1_d_k2 * k1_d_p1);
    let x_ls_1 = alpha_2 * a1.transpose() * k2 / radius_1_sq;
    let x_ls_2 = alpha_1 * a2.transpose() * k1 / radius_2_sq;
    let x_ls = Matrix4x1::new(x_ls_1[0], x_ls_1[1], x_ls_2[0], x_ls_2[1]);

    let n_sym = k1.cross(&k2);
    let pinv_a1 = a1.transpose() / radius_1_sq;
    let pinv_a2 = a2.transpose() / radius_2_sq;

    let a_perp_tilde = Matrix4x3::from_rows(&[pinv_a1.row(0), pinv_a1.row(1), pinv_a2.row(0), pinv_a2.row(1)]) * n_sym;

    let x_ls_2_norm = x_ls.fixed_rows::<2>(0).norm();

    if x_ls_2_norm < 1.0 {
        let xi = (1.0 - x_ls_2_norm * x_ls_2_norm).sqrt() / a_perp_tilde.fixed_slice::<2, 1>(0, 0).norm();
        let sc_1 = x_ls + xi * a_perp_tilde;
        let sc_2 = x_ls - xi * a_perp_tilde;

        let theta1 = SolutionSet2::Two(
            sc_1[0].atan2(sc_1[1]),
            sc_2[0].atan2(sc_2[1]),
        );

        let theta2 = SolutionSet2::Two(
            sc_1[2].atan2(sc_1[3]),
            sc_2[2].atan2(sc_2[3]),
        );

        (theta1, theta2, is_ls)
    }
    else {
        let theta1 = x_ls[0].atan2(x_ls[1]);
        let theta2 = x_ls[2].atan2(x_ls[3]);

        (SolutionSet2::One(theta1), SolutionSet2::One(theta2), true)
    }
}

/**
Solves for `theta1` and `theta2` where `p0 + rot(k1, theta1) * p1 = rot(k2, theta2) * p2`.
Assumes only one solution. If there could be two, `subproblem2` should be used.
*/
pub fn subproblem2extended(p0: &Vector3<f64>, p1: &Vector3<f64>, p2: &Vector3<f64>, k1: &Vector3<f64>, k2: &Vector3<f64>) -> (f64, f64) {
    let kxp1 = k1.cross(p1);
    let kxp2 = k2.cross(p2);

    let a1  = Matrix3x2::from_columns(&[kxp1, -k1.cross(&kxp1)]);
    let a2  = Matrix3x2::from_columns(&[kxp2, -k2.cross(&kxp2)]);
    let a2_neg = -a2;

    let a = Matrix3x4::from_columns(&[a1.column(0), a1.column(1), a2_neg.column(0), a2_neg.column(1)]);

    let p = -k1 * k1.dot(p1) + k2 * k2.dot(p2) - p0;

    let radius1_sq = kxp1.dot(&kxp1);
    let radius2_sq = kxp2.dot(&kxp2);

    let alpha = radius1_sq / (radius1_sq + radius2_sq);
    let beta = radius2_sq / (radius1_sq + radius2_sq);
    let m_inv = Matrix3::identity() + k1 * k1.transpose() * (alpha / (1.0 - alpha));
    let aat_inv = 1.0 / (radius1_sq + radius2_sq) * (m_inv + m_inv * k2 * k2.transpose() * m_inv * beta / (1.0 - (k2.transpose() * m_inv * k2 * beta)[0]));
    let x_ls = a.transpose() * aat_inv * p;

    let n_sym = k1.cross(&k2);
    let pinv_a1 = a1.transpose() / radius1_sq;
    let pinv_a2 = a2.transpose() / radius2_sq;
    let a_perp_tilde = Matrix4x3::from_rows(&[pinv_a1.row(0), pinv_a1.row(1), pinv_a2.row(0), pinv_a2.row(1)]) * n_sym;

    let num = (x_ls.fixed_rows::<2>(2).norm_squared() - 1.0) * a_perp_tilde.fixed_rows::<2>(0).norm_squared() - (x_ls.fixed_rows::<2>(0).norm_squared() - 1.0) * a_perp_tilde.fixed_rows::<2>(2).norm_squared();
    let den = 2.0 * (x_ls.fixed_rows::<2>(0).transpose() * a_perp_tilde.fixed_rows::<2>(0) * a_perp_tilde.fixed_rows::<2>(2).norm_squared() - x_ls.fixed_rows::<2>(2).transpose() * a_perp_tilde.fixed_rows::<2>(2) * a_perp_tilde.fixed_rows::<2>(0).norm_squared())[0];

    let xi = num / den;

    let sc = x_ls + xi * a_perp_tilde;

    (sc[0].atan2(sc[1]), sc[2].atan2(sc[3]))
}

/**
Solves for theta where || rot(k, theta) * p1 - p2 || = d if possibble.
If not, minimizes | || rot(k, theta)*p1 - p2 || - d |.
Also returns a boolean of whether or not theta is a least-squares solution.
*/
pub fn subproblem3(p1: &Vector3<f64>, p2: &Vector3<f64>, k: &Vector3<f64>, d: f64) -> (SolutionSet2<f64>, bool) {
    let kxp = k.cross(p1);
    let a_1 = Matrix3x2::from_columns(&[kxp, -k.cross(&kxp)]);
    let a = -2.0 * p2.transpose() * a_1;
    let norm_a_sq = a.norm_squared();
    let norm_a = a.norm();

    let b = d * d - (p2 - k * k.transpose() * p1).norm_squared() - kxp.norm_squared();

    let x_ls = a_1.transpose() * (-2.0 * p2 * b / norm_a_sq);

    if x_ls.norm_squared() > 1.0 {
        return (SolutionSet2::One(x_ls[0].atan2(x_ls[1])), true);
    }

    let xi = (1.0 - b * b / norm_a_sq).sqrt();

    let a_perp_tilde = Vector2::new(a[1], -a[0]);
    let a_perp = a_perp_tilde / norm_a;

    let sc_1 = x_ls + xi * a_perp;
    let sc_2 = x_ls - xi * a_perp;

    (SolutionSet2::Two(
        sc_1[0].atan2(sc_1[1]),
        sc_2[0].atan2(sc_2[1]),
    ), false)
}

/**
Solves for theta where h' * rot(k, theta) * p = d if possible.
If not minimizes | h' * rot(k, theta) * p - d |.
Also returns a boolean of whether or not theta is a least-squares solution.
 */
pub fn subproblem4(h: &Vector3<f64>, p: &Vector3<f64>, k: &Vector3<f64>, d: f64) -> (SolutionSet2<f64>, bool) {
    let a_11 = k.cross(p);
    let a_1 = Matrix3x2::from_columns(&[a_11, -k.cross(&a_11)]);
    let a = h.transpose() * a_1;

    let b = d - (h.transpose() * k * k.transpose() * p)[0];

    let norm_a_2 = a.norm_squared();

    let x_ls = a_1.transpose() * h * b;

    if norm_a_2 > b * b {
        let xi = (norm_a_2 - b * b).sqrt();
        let a_perp_tilde = Vector2::new(a[1], -a[0]);

        let sc_1 = x_ls + xi * a_perp_tilde;
        let sc_2 = x_ls - xi * a_perp_tilde;

        (
            SolutionSet2::Two(sc_1[0].atan2(sc_1[1]), sc_2[0].atan2(sc_2[1])),
            false,
        )
    }
    else {
        (
            SolutionSet2::One(x_ls[0].atan2(x_ls[1])),
            true,
        )
    }
}

/**
Solves for theta1, theta2, and theta3 where p0 + rot(k1, theta1) * p1 = rot(k2, theta2) * (p2 + rot(k3, theta3) * p3) if possible.
There can be up to 4 solutions.
 */
pub fn subproblem5(p0: &Vector3<f64>, p1: &Vector3<f64>, p2: &Vector3<f64>, p3: &Vector3<f64>, k1: &Vector3<f64>, k2: &Vector3<f64>, k3: &Vector3<f64>) -> (SolutionSet4<f64>, SolutionSet4<f64>, SolutionSet4<f64>) {
    let mut theta1 = [0.0; 8];
    let mut theta2 = [0.0; 8];
    let mut theta3 = [0.0; 8];

    let mut i_soln = 0;

    let p1_s = p0 + k1 * k1.transpose() * p1;
    let p3_s = p2 + k3 * k3.transpose() * p3;

    let delta1 = k2.dot(&p1_s);
    let delta3 = k2.dot(&p3_s);

    let (p_1, r_1) = cone_polynomials(p0, k1, p1, &p1_s, k2);
    let (p_3, r_3) = cone_polynomials(p2, k3, p3, &p3_s, k2);

    let p_13 = p_1 - p_3;
    let p_13_sq = vec_self_convolve_2(&p_13);

    let rhs = r_3 - r_1 - p_13_sq;

    let eqn_real = vec_self_convolve_3(&rhs) - 4.0 * vec_convolve_3(&p_13_sq, &r_1);
    let mut eqn: Vector5<Complex<f64>> = Vector5::zeros();

    for (complex, real) in eqn.iter_mut().zip(eqn_real.into_iter()) {
        complex.re = *real;
    }

    let all_roots = approximate_quartic_roots(eqn).transpose();
    let h_vec = DVector::from_vec(all_roots.into_iter().filter(|c| c.im == 0.0).map(|c| c.re).collect());

    let kxp1 = k1.cross(p1);
    let kxp3 = k3.cross(p3);
    let a_1 = Matrix3x2::from_columns(&[kxp1, -k1.cross(&kxp1)]);
    let a_3 = Matrix3x2::from_columns(&[kxp3, -k3.cross(&kxp3)]);

    let signs_1 = [1.0,  1.0, -1.0, -1.0];
    let signs_3 = [1.0, -1.0,  1.0, -1.0];

    let j = Matrix2::new(
         0.0, 1.0,
        -1.0, 0.0,
    );

    for &h in h_vec.into_iter() {
        let const_1 = a_1.transpose() * k2 * (h - delta1);
        let const_3 = a_3.transpose() * k2 * (h - delta3);
        let pm_1 = j * a_1.transpose() * k2 * ((a_1.transpose() * k2).norm_squared() - (h - delta1).powi(2)).sqrt();
        let pm_3 = j * a_3.transpose() * k2 * ((a_3.transpose() * k2).norm_squared() - (h - delta3).powi(2)).sqrt();

        for (&sign_1, &sign_3) in signs_1.iter().zip(signs_3.iter()) {
            let sc1 = const_1 + sign_1 * pm_1;
            let sc1 = sc1 / (a_1.transpose() * k2).norm_squared();

            let sc3 = const_3 + sign_3 * pm_3;
            let sc3 = sc3 / (a_3.transpose() * k2).norm_squared();

            let v1 = a_1 * sc1 + p1_s;
            let v3 = a_3 * sc3 + p3_s;

            if ((v1 - h * k2).norm() - (v3 - h * k2).norm()).abs() < 1e-5 {
                theta1[i_soln] = sc1[0].atan2(sc1[1]);
                (theta2[i_soln], _) = subproblem1(&v3, &v1, &k2);
                theta3[i_soln] = sc3[0].atan2(sc3[1]);

                i_soln += 1;
            }
        }
    }

    match i_soln {
        1 => (
            SolutionSet4::One(theta1[0]),
            SolutionSet4::One(theta2[0]),
            SolutionSet4::One(theta3[0]),
        ),

        2 => (
            SolutionSet4::Two(theta1[0], theta1[1]),
            SolutionSet4::Two(theta2[0], theta2[1]),
            SolutionSet4::Two(theta3[0], theta3[1]),
        ),

        3 => (
            SolutionSet4::Three(theta1[0], theta1[1], theta1[2]),
            SolutionSet4::Three(theta2[0], theta2[1], theta2[2]),
            SolutionSet4::Three(theta3[0], theta3[1], theta3[2]),
        ),

        4 => (
            SolutionSet4::Four(theta1[0], theta1[1], theta1[2], theta1[3]),
            SolutionSet4::Four(theta2[0], theta2[1], theta2[2], theta2[3]),
            SolutionSet4::Four(theta3[0], theta3[1], theta3[2], theta3[3]),
        ),

        _ => (
            SolutionSet4::Zero,
            SolutionSet4::Zero,
            SolutionSet4::Zero,
        ),
    }
}

fn cone_polynomials(p0_i: &Vector3<f64>, k_i: &Vector3<f64>, p_i: &Vector3<f64>, p_i_s: &Vector3<f64>, k2: &Vector3<f64>) -> (Vector2<f64>, Vector3<f64>) {
    let ki_x_k2 = k_i.cross(k2);
    let ki_x_ki_x_k2 = k_i.cross(&ki_x_k2);
    let norm_ki_x_k2_sq = ki_x_k2.dot(&ki_x_k2);

    let ki_x_pi = k_i.cross(p_i);
    let norm_ki_x_pi_sq = ki_x_pi.dot(&ki_x_pi);

    let alpha = p0_i.transpose() * ki_x_ki_x_k2 / norm_ki_x_k2_sq;
    let delta = k2.dot(p_i_s);
    let beta =  p0_i.transpose() *  ki_x_k2  / norm_ki_x_k2_sq;

    let p_const = norm_ki_x_pi_sq + p_i_s.norm_squared() + 2.0 * alpha[0] * delta;
    let p = Vector2::new(-2.0 * alpha[0], p_const);
/*
    R = [-1 2*delta -delta^2]; % -(H-delta_i)^2
    R(end) = R(end) + norm_kiXpi_sq*norm_kiXk2_sq; % ||A_i' k_2||^2 - (H-delta_i)^2
    R = (2*beta)^2 * R
    */
    let r = Vector3::new(-1.0, 2.0 * delta, -delta * delta + norm_ki_x_pi_sq * norm_ki_x_k2_sq);
    let r = (2.0 * beta[0]).powi(2) * r;

    (p, r)
}

fn vec_self_convolve_2(v: &Vector2<f64>) -> Vector3<f64> {
    let (a, b) = (v[0], v[1]);
    Vector3::new(a * a, 2.0 * a * b, b * b)
}

fn vec_self_convolve_3(v: &Vector3<f64>) -> Vector5<f64> {
    let (a, b, c) = (v[0], v[1], v[2]);
    Vector5::new(a * a, 2.0 * a * b, 2.0 * a * c + b * b, 2.0 * b * c, c * c)
}

fn vec_convolve_3(v1: &Vector3<f64>, v2: &Vector3<f64>) -> Vector5<f64> {
    let (a, b, c) = (v1[0], v1[1], v1[2]);
    let (x, y, z) = (v2[0], v2[1], v2[2]);
    Vector5::new(a * x, b * x + a * y, a * z + b * y + c * x, b * z + c * y, c * z)
}

fn approximate_quartic_roots(p: Vector5<Complex<f64>>) -> Vector4<Complex<f64>> {
    Matrix4::from_columns(&[
        p.fixed_rows::<4>(1) / -p[0],
        Vector4::new(Complex::new(1.0, 0.0), Complex::new(0.0, 0.0), Complex::new(0.0, 0.0), Complex::new(0.0, 0.0)),
        Vector4::new(Complex::new(0.0, 0.0), Complex::new(1.0, 0.0), Complex::new(0.0, 0.0), Complex::new(0.0, 0.0)),
        Vector4::new(Complex::new(0.0, 0.0), Complex::new(0.0, 0.0), Complex::new(1.0, 0.0), Complex::new(0.0, 0.0)),
    ]).transpose().eigenvalues().unwrap()
}
