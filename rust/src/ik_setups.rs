use nalgebra::{Matrix3, Vector3, Vector6};

use crate::{ik_aux::{Kinematics, Matrix3x8, forward_kinematics}, setups::Setup, auxiliary::{random_norm_vector3, random_vector3, random_angle}, inverse_kinematics};

pub struct SphericalToParallelSetup {
    kin: Kinematics,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

impl SphericalToParallelSetup {
    pub fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }
}

impl Setup for SphericalToParallelSetup {
    fn setup(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3())
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        let h_column_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_column_1);

        self.kin.p = Matrix3x8::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = forward_kinematics(&self.kin, &q);
    }

    fn setup_ls(&mut self) {
        todo!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = inverse_kinematics::spherical_to_parallel(&self.r, &self.t, &self.kin);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            let (r_t, t_t) = forward_kinematics(&self.kin, q);
            (r_t - self.r).norm() + (t_t - self.t).norm()
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn is_at_local_min(&self) -> bool {
        todo!()
    }

    fn name(&self) -> &'static str {
        "Spherical to Parallel"
    }
}
