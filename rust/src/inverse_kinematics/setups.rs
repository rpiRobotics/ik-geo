use super::three_parallel_two_intersecting;

use {
    nalgebra::{ Vector3, Vector6, Matrix3 },
    crate::subproblems::{
        setups::{ SetupDynamic, DELTA },

        auxiliary::{
            random_vector3,
            random_norm_vector3,
            random_norm_perp_vector3,
            random_angle,
            rot,
        }
    },

    super::{
        auxiliary::{
            Kinematics,
            Matrix3x7,
            forward_kinematics,
        },

        spherical_two_parallel,
        spherical_two_intersecting,
        spherical,
    },
};

pub struct SphericalTwoParallelSetup {
    kin: Kinematics,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct SphericalTwoIntersectingSetup {
    kin: Kinematics,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct SphericalSetup {
    kin: Kinematics,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct ThreeParallelTwoIntersectingSetup {
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

    fn calculate_error(&self, q: &Vector6<f64>) -> f64 {
        let (r_t, t_t) = forward_kinematics(&self.kin, q.as_slice());
        (r_t - self.r).norm() + (t_t - self.t).norm()
    }
}

impl SphericalTwoIntersectingSetup {
    pub fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn calculate_error(&self, q: &Vector6<f64>) -> f64 {
        let (r_t, t_t) = forward_kinematics(&self.kin, q.as_slice());
        (r_t - self.r).norm() + (t_t - self.t).norm()
    }
}

impl SphericalSetup {
    pub fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn calculate_error(&self, q: &Vector6<f64>) -> f64 {
        let (r_t, t_t) = forward_kinematics(&self.kin, q.as_slice());
        (r_t - self.r).norm() + (t_t - self.t).norm()
    }
}

impl ThreeParallelTwoIntersectingSetup {
    pub fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn calculate_error(&self, q: &Vector6<f64>) -> f64 {
        let (r_t, t_t) = forward_kinematics(&self.kin, q.as_slice());
        (r_t - self.r).norm() + (t_t - self.t).norm()
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

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = forward_kinematics(&self.kin, q.as_slice());
    }

    fn setup_ls(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3())
        }

        let h_column_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_column_1);
        self.kin.h.set_column(0, &random_norm_perp_vector3(&h_column_1));

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        let p_column_1: Vector3<f64> = self.kin.p.column(3).into();
        let p_column_2: Vector3<f64> = self.kin.p.column(3).into();
        let p_column_3: Vector3<f64> = self.kin.p.column(3).into();

        self.kin.p.set_column(3, &(p_column_3 - h_column_1 * (h_column_1.transpose() * (p_column_1 + p_column_2 + p_column_3))));

        let p_column_3: Vector3<f64> = self.kin.p.column(3).into();
        self.kin.h.set_column(4, &random_norm_perp_vector3(&p_column_3));

        let p_column_4: Vector3<f64> = self.kin.p.column(4).into();
        self.kin.h.set_column(5, &random_norm_perp_vector3(&p_column_4));

        self.r = rot(&random_norm_vector3(), random_angle());
        self.t = random_vector3();
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
        self.q.iter().zip(self.is_ls.iter()).map(|(q, &is_ls)| {
            if is_ls {
                0.0
            }
            else {
                self.calculate_error(q)
            }
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn is_at_local_min(&self) -> bool {
        for q in &self.q {
            let error = self.calculate_error(q);
            let mut q_test: Vector6<f64> = q.clone();

            for sign in [-1.0, 1.0] {
                for i in 0..q.len() {
                    q_test[i] += sign * DELTA;

                    if self.calculate_error(&q_test) < error {
                        return false;
                    }

                    q_test[i] = q[i];
                }
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        "Spherical two Parallel"
    }
}

impl SetupDynamic for SphericalTwoIntersectingSetup {
    fn setup(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3())
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            Vector3::zeros(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = forward_kinematics(&self.kin, q.as_slice());
    }

    fn setup_ls(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3())
        }

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            Vector3::zeros(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        self.kin.h.set_column(1, &random_norm_perp_vector3(&self.kin.h.column(0).into()));
        let h_column_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_column_1);
        self.kin.p.set_column(3, &(self.kin.p.column(3) - self.kin.h.column(2) * self.kin.h.column(2).transpose() * self.kin.p.column(2) + self.kin.p.column(3)));

        self.kin.h.set_column(4, &random_norm_perp_vector3(&self.kin.h.column(3).into()));
        self.kin.h.set_column(5, &random_norm_perp_vector3(&self.kin.h.column(4).into()));

        self.r = rot(&random_norm_vector3(), random_angle());
        self.t = random_vector3();
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical_two_intersecting(&self.r, &self.t, &self.kin);
    }

    fn error(&self) -> f64 {
        self.q.iter().zip(self.is_ls.iter()).map(|(q, &is_ls)| {
            if is_ls {
                0.0
            }
            else {
                self.calculate_error(q)
            }
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn is_at_local_min(&self) -> bool {
        for q in &self.q {
            let error = self.calculate_error(q);
            let mut q_test: Vector6<f64> = q.clone();

            for sign in [-1.0, 1.0] {
                for i in 0..q.len() {
                    q_test[i] += sign * DELTA;

                    if self.calculate_error(&q_test) < error {
                        return false;
                    }

                    q_test[i] = q[i];
                }
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        "Spherical Two Intersecting"
    }
}

impl SetupDynamic for SphericalSetup {
    fn setup(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3())
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = forward_kinematics(&self.kin, q.as_slice());
    }

    fn setup_ls(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3());

            self.kin.p = Matrix3x7::from_columns(&[
                random_vector3(),
                random_vector3(),
                random_vector3(),
                random_vector3(),
                Vector3::zeros(),
                Vector3::zeros(),
                random_vector3(),
            ]);

            self.r = rot(&random_norm_vector3(), random_angle());
            self.t = random_vector3();
        }
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical(&self.r, &self.t, &self.kin);
    }

    fn error(&self) -> f64 {
        if self.q.len() == 0 {
            println!("[WARN]\tSubproblem 5 failed to find solutions in IK Spherical");
            return 0.0;
        }

        self.q.iter().zip(self.is_ls.iter()).map(|(q, &is_ls)| {
            if is_ls {
                0.0
            }
            else {
                self.calculate_error(q)
            }
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn is_at_local_min(&self) -> bool {
        for q in &self.q {
            let error = self.calculate_error(q);
            let mut q_test: Vector6<f64> = q.clone();

            for sign in [-1.0, 1.0] {
                for i in 0..q.len() {
                    q_test[i] += sign * DELTA;

                    if self.calculate_error(&q_test) < error {
                        return false;
                    }

                    q_test[i] = q[i];
                }
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        "Spherical"
    }
}

impl SetupDynamic for ThreeParallelTwoIntersectingSetup {
    fn setup(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3())
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        let h_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_1);
        self.kin.h.set_column(3, &h_1);

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            random_vector3(),
            random_vector3(),
        ]);

        (self.r, self.t) = forward_kinematics(&self.kin, q.as_slice());
    }

    fn setup_ls(&mut self) {
        for i in 0..6 {
            self.kin.h.set_column(i, &random_norm_vector3())
        }

        let h_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_1);
        self.kin.h.set_column(3, &h_1);

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        ]);

        let p_4_new = self.kin.p.column(4) - self.kin.h.column(1) * (self.kin.h.column(1).transpose() * (self.kin.p.column(1) + self.kin.p.column(2) + self.kin.p.column(3) + self.kin.p.column(4)));
        self.kin.p.set_column(4, &p_4_new);

        self.kin.h.set_column(0, &random_norm_perp_vector3(&self.kin.h.column(1).into()));
        self.kin.h.set_column(4, &random_norm_perp_vector3(&self.kin.h.column(3).into()));
        self.kin.h.set_column(5, &random_norm_perp_vector3(&self.kin.h.column(4).into()));

        self.r = rot(&random_norm_vector3(), random_angle());
        self.t = random_vector3();
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = three_parallel_two_intersecting(&self.r, &self.t, &self.kin);
    }

    fn error(&self) -> f64 {
        self.q.iter().zip(self.is_ls.iter()).map(|(q, &is_ls)| {
            if is_ls {
                0.0
            }
            else {
                self.calculate_error(q)
            }
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn is_at_local_min(&self) -> bool {
        for q in &self.q {
            let error = self.calculate_error(q);
            let mut q_test: Vector6<f64> = q.clone();

            for sign in [-1.0, 1.0] {
                for i in 0..q.len() {
                    q_test[i] += sign * DELTA;

                    if self.calculate_error(&q_test) < error {
                        return false;
                    }

                    q_test[i] = q[i];
                }
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        "Three Parallel Two Intersecting"
    }
}
