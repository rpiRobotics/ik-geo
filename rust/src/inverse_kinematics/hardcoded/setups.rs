use crate::{inverse_kinematics::{auxiliary::{forward_kinematics_partial, forward_kinematics_general}, spherical_two_intersecting}};

use {
    std::f64::consts::PI,

    nalgebra::{ Vector3, Vector6, Matrix3 },

    crate::{
        inverse_kinematics::{
            auxiliary::{
                Kinematics,
                Matrix3x7,
                forward_kinematics
            },
            spherical_two_parallel,
        },

        subproblems::{ auxiliary::random_angle, setups::SetupDynamic },
    },
};

pub struct Irb6640 {
    kin: Kinematics,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct KukaR800FixedQ3 {
    kin: Kinematics,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    kin_partial: Kinematics,
    r_6t: Matrix3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

impl Irb6640 {
    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    pub fn get_kin() -> Kinematics {
        let mut kin = Kinematics::new();

        let zv = Vector3::new(0.0, 0.0, 0.0);
        let ex = Vector3::new(1.0, 0.0, 0.0);
        let ey = Vector3::new(0.0, 1.0, 0.0);
        let ez = Vector3::new(0.0, 0.0, 1.0);

        kin.h = Matrix3x7::from_columns(&[ez, ey, ey, ex, ey, ex, zv]);
        kin.p = Matrix3x7::from_columns(&[zv, 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ex + 0.2 * ez, zv, zv, 0.2 * ex]);

        kin
    }
}

impl KukaR800FixedQ3 {
    const Q3: f64 = PI / 6.0;

    pub fn new() -> Self {
        let (kin_partial, r_6t) = Self::get_kin_partial();

        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            kin_partial,
            r_6t,

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    pub fn get_kin() -> Kinematics {
        let mut kin = Kinematics::new();

        let zv = Vector3::new(0.0, 0.0, 0.0);
        let ey = Vector3::new(0.0, 1.0, 0.0);
        let ez = Vector3::new(0.0, 0.0, 1.0);

        kin.h = Matrix3x7::from_columns(&[ez, ey, ez, -ey, ez, ey, ez]);
        kin.p = Matrix3x7::from_columns(&[(0.15 + 0.19) * ez, zv, 0.21 * ez, 0.19 * ez, (0.21 + 0.19) * ez, zv, (0.081 + 0.045) * ez]);

        kin
    }

    pub fn get_kin_partial() -> (Kinematics, Matrix3<f64>) {
        let kin = Self::get_kin();
        forward_kinematics_partial(&kin, Self::Q3, 2, &Matrix3::identity())
    }
}

impl SetupDynamic for Irb6640 {
    fn setup(&mut self) {
        let q = Vector6::zeros().map(|_: f64| random_angle());
        (self.r, self.t) = forward_kinematics(&self.kin, &q);
    }

    fn setup_ls(&mut self) {
        unimplemented!()
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical_two_parallel(&self.r, &self.t, &self.kin);
    }

    fn error(&self) -> f64 {
        self.q.iter().zip(self.is_ls.iter()).map(|(q, &is_ls)| {
            if is_ls {
                0.0
            }
            else {
                let (r_t, t_t) = forward_kinematics(&self.kin, q);
                (r_t - self.r).norm() + (t_t - self.t).norm()
            }
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn is_at_local_min(&self) -> bool {
        unimplemented!()
    }

    fn name(&self) -> &'static str {
        "IRB 6640"
    }
}

impl SetupDynamic for KukaR800FixedQ3 {
    fn setup(&mut self) {
        let mut q = Vector6::zeros().map(|_: f64| random_angle());
        q[2] = Self::Q3;
        (self.r, self.t) = forward_kinematics(&self.kin, &q);
    }

    fn setup_ls(&mut self) {
        unimplemented!()
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical_two_intersecting(&(self.r * self.r_6t.transpose()), &self.t, &self.kin_partial);
    }

    fn error(&self) -> f64 {
        self.q.iter().zip(self.is_ls.iter()).map(|(q, &is_ls)| {
            if is_ls {
                0.0
            }
            else {
                let q_e = vec![
                    q[0],
                    q[1],
                    q[2],
                    1.0,
                    q[3],
                    q[4],
                    q[5],
                ];

                let (r_t, t_t) = forward_kinematics_general(&self.kin, &q_e);
                (r_t - self.r).norm() + (t_t - self.t).norm()
            }
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn is_at_local_min(&self) -> bool {
        unimplemented!()
    }

    fn name(&self) -> &'static str {
        "KUKA R800 Fixed Q3"
    }
}
