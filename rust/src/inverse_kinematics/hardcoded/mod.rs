pub(crate) mod setups;

use {
    self::setups::{ Irb6640, KukaR800FixedQ3, Ur5, ThreeParallelBot, TwoParallelBot, RrcFixedQ6, SphericalBot, YumiFixedQ3 },
    super::{ spherical_two_parallel, spherical_two_intersecting, three_parallel_two_intersecting, three_parallel, two_parallel, two_intersecting, spherical, gen_six_dof },
    nalgebra::{ Matrix3, Vector3, Vector6 },
};

pub fn irb6640(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    spherical_two_parallel(r, t, &Irb6640::get_kin())
}

pub fn kuka_r800_fixed_q3(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let (kin_partial, r_6t) = KukaR800FixedQ3::get_kin_partial();
    spherical_two_intersecting(&(r * r_6t.transpose()), t, &kin_partial)
}

pub fn rrc_fixed_q6(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let (kin_partial, r_6t) = RrcFixedQ6::get_kin_partial();
    two_intersecting(&(r * r_6t.transpose()), t, &kin_partial)
}

pub fn yumi_fixed_q3(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let (kin_partial, r_6t) = YumiFixedQ3::get_kin_partial();
    gen_six_dof(&(r * r_6t.transpose()), t, &kin_partial)
}

pub fn ur5(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    three_parallel_two_intersecting(r, t, &Ur5::get_kin())
}

pub fn three_parallel_bot(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    three_parallel(r, t, &ThreeParallelBot::get_kin())
}

pub fn two_parallel_bot(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    two_parallel(r, t, &TwoParallelBot::get_kin())
}

pub fn spherical_bot(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    spherical(&r, &t, &SphericalBot::get_kin())
}
