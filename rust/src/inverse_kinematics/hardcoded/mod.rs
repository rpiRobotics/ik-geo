use self::setups::KukaR800FixedQ3;

use super::spherical_two_intersecting;

pub mod setups;

use {
    self::setups::Irb6640,
    super::spherical_two_parallel,
    nalgebra::{Matrix3, Vector3, Vector6},
};

pub fn irb6640(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    spherical_two_parallel(r, t, &Irb6640::get_kin())
}

pub fn kuka_r800_fixed_q3(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let (kin_partial, r_6t) = KukaR800FixedQ3::get_kin_partial();
    spherical_two_intersecting(&(r * r_6t.transpose()), &t, &kin_partial)
}
