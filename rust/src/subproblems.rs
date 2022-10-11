use nalgebra::{ Vector3, Matrix3x2, Matrix4x3, Matrix4x1, DVector };

/// Solves for theta where rot(k, theta) * p1 = p2 if possible.
/// If not, minimizes || rot(k, theta) * p1 - p2 ||
/// Also returns a boolean of whether or not theta is a least-squares solution
pub fn subproblem1(p1: Vector3<f64>, p2: Vector3<f64>, k: Vector3<f64>) -> (f64, bool) {
    let kxp = k.cross(&p1);
    let a = Matrix3x2::from_columns(&[kxp, -k.cross(&kxp)]);
    let x = a.transpose() * p2;

    let theta = x[0].atan2(x[1]);
    let is_ls = (p1.norm() - p2.norm()).abs() > 1e-8 || (k.dot(&p1) - k.dot(&p2)).abs() > 1e-8;

    (theta, is_ls)
}

/// Solves for theta1 and theta2 where rot(k1, theta1)*p1 = rot(k2, theta2)*p2 if possible
/// If not, minimizes || rot(k1, theta1)*p1 - rot(k2, theta2)*p2 ||
/// Also returns a boolean of whether or not { theta1, theta2 } is a least-squares solution
/// There may be 1 or 2 solutions for theta1 and theta2 so they are returned as 1 or 2 sized dynamic vectors
pub fn subproblem2(p1: Vector3<f64>, p2: Vector3<f64>, k1: Vector3<f64>, k2: Vector3<f64>) -> (DVector<f64>, DVector<f64>, bool) {
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

        let theta1 = DVector::from_vec(vec![
            sc_1[0].atan2(sc_1[1]),
            sc_2[0].atan2(sc_2[1]),
        ]);

        let theta2 = DVector::from_vec(vec![
            sc_1[2].atan2(sc_1[3]),
            sc_2[2].atan2(sc_2[3]),
        ]);

        (theta1, theta2, is_ls)
    }
    else {
        let theta1 = x_ls[0].atan2(x_ls[1]);
        let theta2 = x_ls[2].atan2(x_ls[3]);

        (DVector::from_vec(vec![theta1]), DVector::from_vec(vec![theta2]), true)
    }
}
