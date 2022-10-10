use nalgebra::{ Vector3, Matrix3x2 };

/// Solves for theta where rot(k, theta) * p1 = p2 if possible.
/// If not, minimizes || rot(k, theta) * p1 - p2 ||
pub fn subproblem1(p1: Vector3<f64>, p2: Vector3<f64>, k: Vector3<f64>) -> (f64, bool) {
    let kxp = k.cross(&p1);
    let a = Matrix3x2::from_columns(&[kxp, -k.cross(&kxp)]);
    let x = a.transpose() * p2;

    let theta = x[0].atan2(x[1]);
    let is_ls = (p1.norm() - p2.norm()).abs() > 1e-8 || (k.dot(&p1) - k.dot(&p2)).abs() > 1e-8;

    (theta, is_ls)
}
