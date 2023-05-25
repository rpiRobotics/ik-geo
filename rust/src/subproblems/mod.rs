use std::f64::INFINITY;

pub(crate) mod auxiliary;
pub(crate) mod setups;

use {
    super::solutionset::{ SolutionSet2, SolutionSet4 },

    auxiliary::{
        vec_self_convolve_2,
        vec_self_convolve_3,
        vec_convolve_3,
        solve_two_ellipse_numeric,
        cone_polynomials,
        solve_quartic_roots,
        solve_lower_triangular_system_2x2,
    },

    nalgebra::{
        Vector2, Vector3, Vector4, Vector5,
        Matrix, Matrix3, Matrix4, Matrix2x4, Matrix3x2, Matrix3x4, Matrix4x3,
        ArrayStorage, U1, U7,
        Complex, DVector, Matrix2,
    },
};

pub(crate) type Vector7<T> = Matrix<T, U7, U1, ArrayStorage<T, 7, 1>>;

/// Solves for `theta` where `rot(k, theta) * p1 = p2` if possible.
/// If not, minimizes `|| rot(k, theta) * p1 - p2 ||`.
/// Also returns a boolean of whether or not `theta` is a least-squares solution.
pub fn subproblem1(p1: &Vector3<f64>, p2: &Vector3<f64>, k: &Vector3<f64>) -> (f64, bool) {
    let kxp = k.cross(p1);
    let a = Matrix3x2::from_columns(&[kxp, -k.cross(&kxp)]);
    let x = a.transpose() * p2;

    let theta = x[0].atan2(x[1]);
    let is_ls = (p1.norm() - p2.norm()).abs() > 1e-6 || (k.dot(p1) - k.dot(p2)).abs() > 1e-6;

    (theta, is_ls)
}

// Solves for `theta1` and `theta2` where `rot(k1, theta1) * p1 = rot(k2, theta2) * p2` if possible.
// If not, minimizes `|| rot(k1, theta1) * p1 - rot(k2, theta2) * p2 ||`.
// Also returns a boolean of whether or not `{ theta1, theta2 }` is a least-squares solution.
// There may be 1 or 2 solutions for `theta1` and `theta2`.
pub fn subproblem2(p1: &Vector3<f64>, p2: &Vector3<f64>, k1: &Vector3<f64>, k2: &Vector3<f64>) -> (SolutionSet2<(f64, f64)>, bool) {
    let p1_norm = p1.normalize();
    let p2_norm = p2.normalize();

    let (theta1, theta1_is_ls) = subproblem4(k2, &p1_norm, k1, k2.dot(&p2_norm));
    let (theta2, theta2_is_ls) = subproblem4(k1, &p2_norm, k2, k1.dot(&p1_norm));

    let is_ls = (p1.norm() - p2.norm()).abs() > 1e-8 || theta1_is_ls || theta2_is_ls;

    // Reverse theta2 and duplicate any angle with less solutions
    let solution = if theta1.size() > 1 || theta2.size() > 1 {
        let (theta1_first, theta1_second) = theta1.duplicated().expect_two();
        let (theta2_first, theta2_second) = theta2.duplicated().expect_two();

        SolutionSet2::Two(
            (theta1_first, theta2_second),
            (theta1_second, theta2_first),
        )
    }
    else {
        SolutionSet2::One(
            (theta1.expect_one(), theta2.expect_one()),
        )
    };

    (solution, is_ls)
}

// Solves for `theta1` and `theta2` where `p0 + rot(k1, theta1) * p1 = rot(k2, theta2) * p2`.
// Assumes only one solution. If there could be two, `subproblem2` should be used.
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

// Solves for `theta` where `|| rot(k, theta) * p1 - p2 || = d` if possibble.
// If not, minimizes `| || rot(k, theta)*p1 - p2 || - d |`.
// Also returns a boolean of whether or not `theta` is a least-squares solution.
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

/// Solves for `theta` where `h' * rot(k, theta) * p = d` if possible.
/// If not minimizes `| h' * rot(k, theta) * p - d |`.
/// Also returns a boolean of whether or not `theta` is a least-squares solution.
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

/// Solves for `theta1`, `theta2`, and `theta3` where `p0 + rot(k1, theta1) * p1 = rot(k2, theta2) * (p2 + rot(k3, theta3) * p3)` if possible.
/// There can be up to 4 solutions.
pub fn subproblem5(p0: &Vector3<f64>, p1: &Vector3<f64>, p2: &Vector3<f64>, p3: &Vector3<f64>, k1: &Vector3<f64>, k2: &Vector3<f64>, k3: &Vector3<f64>) -> SolutionSet4<(f64, f64, f64)> {
    /// Given n >= 4 solutions, return the top 4 most unique
    fn reduced_solutionset(mut solutions: Vec<(f64, f64, f64)>) -> SolutionSet4<(f64, f64, f64)> {
        if solutions.len() <= 4 {
            return SolutionSet4::from_vec(&solutions);
        }

        solutions.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());

        let mut solutions_with_ordering = Vec::with_capacity(solutions.len());
        let mut last = INFINITY;

        for solution in solutions {
            let delta = solution.0 - last;
            let ordering = 1.0 / (delta * delta);

            solutions_with_ordering.push((ordering, solution));
            last = solution.0;
        }

        solutions_with_ordering.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());

        SolutionSet4::Four(
            solutions_with_ordering[0].1,
            solutions_with_ordering[1].1,
            solutions_with_ordering[2].1,
            solutions_with_ordering[3].1,
        )
    }

    const EPSILON: f64 = 1e-6;

    let mut theta = Vec::with_capacity(8);

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

    let all_roots = solve_quartic_roots(&eqn).transpose();
    let h_vec = DVector::from_vec(all_roots.into_iter().filter(|c| c.im.abs() < EPSILON).map(|c| c.re).collect());

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

        let hd1 = h - delta1;
        let hd3 = h - delta3;

        let sq1 = (a_1.transpose() * k2).norm_squared() - hd1 * hd1;
        if sq1 < 0.0 { continue; }

        let sq3 = (a_3.transpose() * k2).norm_squared() - hd3 * hd3;
        if sq3 < 0.0 { continue; }

        let pm_1 = j * a_1.transpose() * k2 * sq1.sqrt();
        let pm_3 = j * a_3.transpose() * k2 * sq3.sqrt();

        for (&sign_1, &sign_3) in signs_1.iter().zip(signs_3.iter()) {
            let sc1 = const_1 + sign_1 * pm_1;
            let sc1 = sc1 / (a_1.transpose() * k2).norm_squared();

            let sc3 = const_3 + sign_3 * pm_3;
            let sc3 = sc3 / (a_3.transpose() * k2).norm_squared();

            let v1 = a_1 * sc1 + p1_s;
            let v3 = a_3 * sc3 + p3_s;

            if ((v1 - h * k2).norm() - (v3 - h * k2).norm()).abs() < 1e-6 {
                let (theta2_value, _) = subproblem1(&v3, &v1, &k2);

                theta.push((
                    sc1[0].atan2(sc1[1]),
                    theta2_value,
                    sc3[0].atan2(sc3[1]),
                ));
            }
        }
    }

    reduced_solutionset(theta)
}

/// Solves for `theta1` and `theta2` where `h1' * rot(k1, theta1) + h2' * rot(k2, theta2) = d1` and `h3' * rot(k3, theta1) + h4' * rot(k4, theta2) = d2`
/// There can be up to 4 solutions
pub fn subproblem6(h: &[Vector3<f64>; 4], k: &[Vector3<f64>; 4], p: &[Vector3<f64>; 4], d1: f64, d2: f64) -> SolutionSet4<(f64, f64)> {
    let k1xp1 = k[0].cross(&p[0]);
    let k2xp2 = k[1].cross(&p[1]);
    let k3xp3 = k[2].cross(&p[2]);
    let k4xp4 = k[3].cross(&p[3]);

    let a1 = Matrix3x2::from_columns(&[k1xp1, -k[0].cross(&k1xp1)]);
    let a2 = Matrix3x2::from_columns(&[k2xp2, -k[1].cross(&k2xp2)]);
    let a3 = Matrix3x2::from_columns(&[k3xp3, -k[2].cross(&k3xp3)]);
    let a4 = Matrix3x2::from_columns(&[k4xp4, -k[3].cross(&k4xp4)]);

    let h1_a1 = h[0].transpose() * a1;
    let h2_a2 = h[1].transpose() * a2;
    let h3_a3 = h[2].transpose() * a3;
    let h4_a4 = h[3].transpose() * a4;

    let a = Matrix2x4::from_rows(&[
        Vector4::new(h1_a1[0], h1_a1[1], h2_a2[0], h2_a2[1]).transpose(),
        Vector4::new(h3_a3[0], h3_a3[1], h4_a4[0], h4_a4[1]).transpose(),
    ]);

    let b = Vector2::new(
        d1 - (h[0].transpose() * k[0] * k[0].transpose() * p[0])[0] - (h[1].transpose() * k[1] * k[1].transpose() * p[1])[0],
        d2 - (h[2].transpose() * k[2] * k[2].transpose() * p[2])[0] - (h[3].transpose() * k[3] * k[3].transpose() * p[3])[0],
    );

    let mut q = Matrix4::identity();
    let qr = a.transpose().qr();

    qr.q_tr_mul(&mut q); // small hack to get entire q matrix
    q = q.transpose();

    let (x_null_1, x_null_2): (Vector4<f64>, Vector4<f64>) = (q.column(2).into(), q.column(3).into());
    let q = q.fixed_columns::<2>(0);
    let r = qr.r().transpose();
    let x_min = q * solve_lower_triangular_system_2x2(&r, &b);

    let xi_i = solve_two_ellipse_numeric(
        &x_min.fixed_rows::<2>(0).into(),
        &Matrix2::new(x_null_1[0], x_null_2[0], x_null_1[1], x_null_2[1]),
        &x_min.fixed_rows::<2>(2).into(),
        &Matrix2::new(x_null_1[2], x_null_2[2], x_null_1[3], x_null_2[3]),
    ).get_all();

    let mut theta = vec![(0.0, 0.0); xi_i.len()];

    for (i, xi) in xi_i.iter().enumerate() {
        let x = x_min + x_null_1 * xi.0 + x_null_2 * xi.1;

        theta[i] = (x[0].atan2(x[1]), x[2].atan2(x[3]));
    }

    SolutionSet4::from_vec(&theta)
}
