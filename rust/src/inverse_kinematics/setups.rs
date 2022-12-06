use {
    nalgebra::{ Vector3, Vector6, Matrix3 },
    crate::subproblems::{
        setups::SetupDynamic,

        auxiliary::{
            random_vector3,
            random_norm_vector3,
            random_angle,
        }
    },

    super::{
        auxiliary::{
            Kinematics,
            Matrix3x8,
            forward_kinematics,
        },

        spherical_two_parallel,
    },
};

pub struct SphericalTwoParallelSetup {
    kin: Kinematics,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

impl SphericalTwoParallelSetup {
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

impl SetupDynamic for SphericalTwoParallelSetup {
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

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!();
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical_two_parallel(&self.r, &self.t, &self.kin);
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
