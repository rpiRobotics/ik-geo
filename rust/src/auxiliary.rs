use nalgebra::{ Vector3, Matrix3 };

/// Creates a 3x3 rotation matrix
pub fn rot(k: Vector3<f64>, theta: f64) -> Matrix3<f64> {
    let k = k.normalize();
    Matrix3::identity() + hat(k) * theta.sin() + hat(k) * hat(k) * (1.0 - theta.cos())
}

pub fn random_vector3() -> Vector3<f64> {
    Vector3::new(random(), random(), random())
}

pub fn random_norm_vector3() -> Vector3<f64> {
    Vector3::new(random(), random(), random()).normalize()
}

pub fn random_angle() -> f64 {
    random() * std::f64::consts::PI
}

fn random() -> f64 {
    fastrand::f64() * 2.0 - 1.0
}

// Matrix cross-product for a 3 x 3 vector
fn hat(k: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -k.z, k.y,
        k.z, 0.0, -k.x,
        -k.y, k.x, 0.0,
    )
}
