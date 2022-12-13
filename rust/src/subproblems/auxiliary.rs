use {
    crate::solutionset::{ SolutionSet4, SolutionSet2 },

    nalgebra::{
        ComplexField,
        Complex,
        DVector,
        Vector3,
        Vector2,
        Vector4,
        Vector5,
        Matrix2,
        Matrix2x4,
        Matrix3,
        Matrix4
    },
};

/// Creates a 3x3 rotation matrix
pub fn rot(k: &Vector3<f64>, theta: f64) -> Matrix3<f64> {
    let k = k.normalize();
    Matrix3::identity() + hat(&k) * theta.sin() + hat(&k) * hat(&k) * (1.0 - theta.cos())
}

pub fn random_vector3() -> Vector3<f64> {
    Vector3::new(random(), random(), random())
}

pub fn random_norm_vector3() -> Vector3<f64> {
    Vector3::new(random(), random(), random()).normalize()
}

pub fn random_norm_perp_vector3(reference: &Vector3<f64>) -> Vector3<f64> {
    random_vector3().cross(&reference).normalize()
}

pub fn random_angle() -> f64 {
    random() * std::f64::consts::PI
}

pub fn cone_polynomials(p0_i: &Vector3<f64>, k_i: &Vector3<f64>, p_i: &Vector3<f64>, p_i_s: &Vector3<f64>, k2: &Vector3<f64>) -> (Vector2<f64>, Vector3<f64>) {
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

    let r = Vector3::new(-1.0, 2.0 * delta, -delta * delta + norm_ki_x_pi_sq * norm_ki_x_k2_sq);
    let r = (2.0 * beta[0]).powi(2) * r;

    (p, r)
}

pub fn vec_self_convolve_2(v: &Vector2<f64>) -> Vector3<f64> {
    let (a, b) = (v[0], v[1]);
    Vector3::new(a * a, 2.0 * a * b, b * b)
}

pub fn vec_self_convolve_3(v: &Vector3<f64>) -> Vector5<f64> {
    let (a, b, c) = (v[0], v[1], v[2]);
    Vector5::new(a * a, 2.0 * a * b, 2.0 * a * c + b * b, 2.0 * b * c, c * c)
}

pub fn vec_convolve_3(v1: &Vector3<f64>, v2: &Vector3<f64>) -> Vector5<f64> {
    let (a, b, c) = (v1[0], v1[1], v1[2]);
    let (x, y, z) = (v2[0], v2[1], v2[2]);
    Vector5::new(a * x, b * x + a * y, a * z + b * y + c * x, b * z + c * y, c * z)
}

pub fn approximate_quartic_roots(p: &Vector5<Complex<f64>>) -> Vector4<Complex<f64>> {
    Matrix4::from_columns(&[
        p.fixed_rows::<4>(1) / -p[0],
        Vector4::new(Complex::new(1.0, 0.0), Complex::new(0.0, 0.0), Complex::new(0.0, 0.0), Complex::new(0.0, 0.0)),
        Vector4::new(Complex::new(0.0, 0.0), Complex::new(1.0, 0.0), Complex::new(0.0, 0.0), Complex::new(0.0, 0.0)),
        Vector4::new(Complex::new(0.0, 0.0), Complex::new(0.0, 0.0), Complex::new(1.0, 0.0), Complex::new(0.0, 0.0)),
    ]).transpose().eigenvalues().unwrap()
}

pub fn null_space_matrix2x4(a: &Matrix2x4<f64>, epsilon: f64) -> SolutionSet2<Vector4<f64>> {
    let eigen = (a.transpose() * a).symmetric_eigen();

    let mut zero_indices = [0, 0];
    let mut i = 0;

    for (j, v) in eigen.eigenvalues.iter().enumerate() {
        if v.abs() < epsilon {
            zero_indices[i] = j;
            i += 1;

            if i >= zero_indices.len() {
                break;
            }
        }
    }

    match i {
        0 => SolutionSet2::Zero,
        2 => SolutionSet2::Two(eigen.eigenvectors.column(zero_indices[0]).into() , eigen.eigenvectors.column(zero_indices[1]).into()),
        _ => unreachable!(),
    }
}

pub fn solve_2_ellipse_numeric(xm1: &Vector2<f64>, xn1: &Matrix2<f64>, xm2: &Vector2<f64>, xn2: &Matrix2<f64>) -> SolutionSet4<(f64, f64)> {
    let a_1 = xn1.transpose() * xn1;
    let a = a_1[0];
    let b = 2.0 * a_1[(1, 0)];
    let c = a_1[(1, 1)];
    let b_1 = 2.0 * xm1.transpose() * xn1;
    let d = b_1[0];
    let e = b_1[1];
    let f = (xm1.transpose() * xm1)[0] - 1.0;

    let a_2 = xn2.transpose() * xn2;
    let a1 = a_2[0];
    let b1 = 2.0 * a_2[(1, 0)];
    let c1 = a_2[(1, 1)];
    let b_2 = 2.0 * xm2.transpose() * xn2;
    let d1 = b_2[0];
    let e1 = b_2[1];
    let fq = (xm2.transpose() * xm2)[0] - 1.0;

    let z0 = f * a * (d1 * d1) + (a * a) * (fq * fq) - d * a * d1 * fq + (a1*a1) * (f * f)
        - 2.0 * a * fq * a1 * f - d * d1 * a1 * f + a1 * (d * d) * fq;

    let z1 = e1 * (d * d) * a1 - fq * d1 * a * b - 2.0 * a * fq * a1 * e - f * a1 * b1 * d
        + 2.0 * d1 * b1 * a * f + 2.0 * e1 * fq * (a * a) + (d1 * d1) * a * e - e1 * d1 * a * d
        - 2.0 * a * e1 * a1 * f - f * a1 * d1 * b + 2.0 * f * e * (a1 * a1) - fq * b1 * a * d - e
        * a1 * d1 * d + 2.0 * fq * b * a1 * d;

    let z2 = (e1 * e1) * (a * a) + 2.0 * c1 * fq * (a * a) - e * a1 * d1 * b + fq * a1
        * (b * b) - e * a1 * b1 * d - fq * b1 * a * b - 2.0 * a * e1 * a1 * e + 2.0 * d1 * b1 * a
        * e - c1 * d1 * a * d - 2.0 * a * c1 * a1 * f + (b1 * b1) * a * f + 2.0 * e1 * b * a1 * d
        + (e * e) * (a1 * a1) - c * a1 * d1 * d - e1 * b1 * a * d + 2.0 * f * c * (a1 * a1) - f * a1
        * b1 * b + c1 * (d * d) * a1 + (d1 * d1) * a * c - e1 * d1 * a * b - 2.0 * a * fq * a1 * c;

    let z3 = -2.0 * a * a1 * c * e1 + e1 * a1 * (b * b) + 2.0 * c1 * b * a1 * d - c * a1 * b1
        * d + (b1 * b1) * a * e - e1 * b1 * a * b - 2.0 * a * c1 * a1 * e - e * a1 * b1 * b - c1
        * b1 * a * d + 2.0 * e1 * c1 * (a * a) + 2.0 * e * c * (a1 * a1) - c * a1 * d1 * b + 2.0
        * d1 * b1 * a * c - c1 * d1 * a * b;

    let z4 = (a * a) * (c1 * c1) - 2.0 * a * c1 * a1 * c + (a1 * a1) * (c * c) - b * a * b1
        * c1 - b * b1 * a1 * c + (b * b) * a1 * c1 + c * a * (b1 * b1);

    let y = approximate_quartic_roots(&Vector5::new(
        Complex::from_real(z4),
        Complex::from_real(z3),
        Complex::from_real(z2),
        Complex::from_real(z1),
        Complex::from_real(z0)
    ));

    let y_sq = y.iter().filter(|z| z.im == 0.0).map(|z| z.re * z.re).collect::<Vec<f64>>();
    let y = y.iter().filter(|z| z.im == 0.0).map(|z| z.re).collect::<Vec<f64>>();
    let y_sq = DVector::from_vec(y_sq);
    let y = DVector::from_vec(y);


    let x = -(((a * c1 * y_sq.clone()).add_scalar(a * fq) - a1 * c * y_sq.clone() + a * e1 * y.clone() - a1 * e * y.clone()).add_scalar(-a1 * f)).component_div(&(((a * b1 * y.clone()).add_scalar(a * d1) - a1 * b * y.clone()).add_scalar(-a1 * d)));

    SolutionSet4::from_vec(&x.iter().zip(y.iter()).map(|(u, v)| (*u, *v)).collect::<Vec<(f64, f64)>>())
}

fn random() -> f64 {
    fastrand::f64() * 2.0 - 1.0
}

// Matrix cross-product for a 3 x 3 vector
fn hat(k: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -k.z, k.y,
        k.z, 0.0, -k.x,
        -k.y, k.x, 0.0,
    )
}
