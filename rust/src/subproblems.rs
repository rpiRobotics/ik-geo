use nalgebra::{ Vector3, Matrix3x2, Matrix4x3, Matrix4x1, Matrix3x4, Matrix3, Vector2 };

#[derive(Clone)]
pub enum SolutionSet2<T> {
    One(T),
    Two(T, T),
}

impl<T> SolutionSet2<T> where T: Copy {
    pub fn expect_one(&self) -> T {
        match self {
            Self::One(s) => *s,
            Self::Two(..) => panic!("Found two solutions where one was expected"),
        }
    }

    pub fn expect_two(&self) -> [T; 2] {
        match self {
            Self::One(_) => panic!("Found one solution where two were expected"),
            Self::Two(s1, s2) => [*s1, *s2],
        }
    }

    pub fn get_first(&self) -> T {
        match self {
            Self::One(s) => *s,
            Self::Two(s, _) => *s,
        }
    }

    pub fn get_all(&self) -> Vec<T> {
        match self {
            Self::One(s) => vec![*s],
            Self::Two(s1, s2) => vec![*s1, *s2],
        }
    }
}

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
