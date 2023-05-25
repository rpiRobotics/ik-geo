use {
    crate::{
        subproblems::{
            subproblem1,
            subproblem2,
            subproblem2extended,
            subproblem3,
            subproblem4,
            subproblem5,
            subproblem6,
        },

        solutionset::{ SolutionSet2, SolutionSet4 },
    },

    super::auxiliary::{
        random_vector3,
        random_norm_vector3,
        random_angle,
        rot,
    },

    nalgebra::{ Vector3, Vector2 },
};

pub const DELTA: f64 = 1e-12;

/// An interface for setting up subproblem testing. Resposible for generating parameters, running
/// the function, and calculating data such as the error.
pub trait SetupDynamic {
    /// Initialize parameters to test the case where theta is solved for exactly
    fn setup(&mut self);

    /// Initialize parameters to test the case where theta is minimized using least squares
    fn setup_ls(&mut self);

    fn setup_from_str(&mut self, raw: &str);
    fn write_output(&self) -> String;

    fn run(&mut self);
    fn run_report_info(&mut self) -> bool;

    fn error(&self) -> f64;
    fn is_at_local_min(&self) -> bool;
    fn name(&self) -> &'static str;
}

pub trait SetupStatic {
    fn new() -> Self;
    fn name() -> &'static str;
}

pub struct Subproblem1Setup {
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    k: Vector3<f64>,

    theta: f64,
}

pub struct Subproblem2Setup {
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    k1: Vector3<f64>,
    k2: Vector3<f64>,

    theta: SolutionSet2<(f64, f64)>,
}

pub struct Subproblem2ExtendedSetup {
    p0: Vector3<f64>,
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    k1: Vector3<f64>,
    k2: Vector3<f64>,

    theta1: f64,
    theta2: f64,
}


pub struct Subproblem3Setup {
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    k: Vector3<f64>,
    d: f64,

    theta: SolutionSet2<f64>,
}

pub struct Subproblem4Setup {
    h: Vector3<f64>,
    p: Vector3<f64>,
    k: Vector3<f64>,
    d: f64,

    theta: SolutionSet2<f64>,
}

pub struct Subproblem5Setup {
    p0: Vector3<f64>,
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    p3: Vector3<f64>,

    k1: Vector3<f64>,
    k2: Vector3<f64>,
    k3: Vector3<f64>,

    theta: SolutionSet4<(f64, f64, f64)>,
}

pub struct Subproblem6Setup {
    h: [Vector3<f64>; 4],
    k: [Vector3<f64>; 4],
    p: [Vector3<f64>; 4],

    d1: f64,
    d2: f64,

    theta: SolutionSet4<(f64, f64)>,
}

impl SetupStatic for Subproblem1Setup {
    fn new() -> Self {
        Self {
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            k: Vector3::zeros(),

            theta: 0.0,
        }
    }

    fn name() -> &'static str {
        "Subproblem 1"
    }
}

impl Subproblem1Setup {
    fn calculate_error(&self, theta: f64) -> f64 {
        (self.p2 - rot(&self.k, theta) * self.p1).norm()
    }
}

impl SetupStatic for Subproblem2Setup {
    fn new() -> Self {
        Self {
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            k1: Vector3::zeros(),
            k2: Vector3::zeros(),

            theta: SolutionSet2::One((0.0, 0.0)),
        }
    }

    fn name() -> &'static str {
        "Subproblem 2"
    }
}

impl Subproblem2Setup {
    fn calculate_error(&self, (t1, t2): (f64, f64)) -> f64 {
        (rot(&self.k2, t2) * self.p2 - rot(&self.k1, t1) * self.p1).norm()
    }
}

impl SetupStatic for Subproblem2ExtendedSetup {
    fn new() -> Self {
        Self {
            p0: Vector3::zeros(),
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            k1: Vector3::zeros(),
            k2: Vector3::zeros(),

            theta1: 0.0,
            theta2: 0.0,
        }
    }

    fn name() -> &'static str {
        "Subproblem 2 Ex"
    }
}

impl SetupStatic for Subproblem3Setup {
    fn new() -> Self {
        Self {
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            k: Vector3::zeros(),
            d: 0.0,

            theta: SolutionSet2::One(0.0),
        }
    }

    fn name() -> &'static str {
        "Subproblem 3"
    }
}

impl Subproblem3Setup {
    fn calculate_error(&self, theta: &[f64]) -> f64 {
        theta
            .iter()
            .map(|&t| {
                ((self.p2 - rot(&self.k, t) * self.p1).norm() - self.d).abs()
            })
            .sum()
    }
}

impl SetupStatic for Subproblem4Setup {
    fn new() -> Self {
        Self {
            h: Vector3::zeros(),
            p: Vector3::zeros(),
            k: Vector3::zeros(),
            d: 0.0,

            theta: SolutionSet2::One(0.0),
        }
    }

    fn name() -> &'static str {
        "Subproblem 4"
    }
}

impl Subproblem4Setup {
    fn calculate_error(&self, theta: &[f64]) -> f64 {
        theta
            .iter()
            .map(|&t| {
                ((self.h.transpose() * rot(&self.k, t) * self.p)[0] - self.d).abs()
            })
            .sum::<f64>() / theta.len() as f64
    }
}

impl SetupStatic for Subproblem5Setup {
    fn new() -> Self {
        Self {
            p0: Vector3::zeros(),
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            p3: Vector3::zeros(),

            k1: Vector3::zeros(),
            k2: Vector3::zeros(),
            k3: Vector3::zeros(),

            theta: SolutionSet4::One((0.0, 0.0, 0.0)),
        }
    }

    fn name() -> &'static str {
        "Subproblem 5"
    }
}

impl SetupStatic for Subproblem6Setup {
    fn new() -> Self {
        Self {
            h: [Vector3::zeros(); 4],
            k: [Vector3::zeros(); 4],
            p: [Vector3::zeros(); 4],

            d1: 0.0,
            d2: 0.0,

            theta: SolutionSet4::One((0.0, 0.0)),
        }
    }

    fn name() -> &'static str {
        "Subproblem 6"
    }
}

impl SetupDynamic for Subproblem1Setup {
    fn setup(&mut self) {
        self.p1 = random_vector3();
        self.k = random_norm_vector3();
        self.theta = random_angle();

        self.p2 = rot(&self.k, self.theta) * self.p1;
    }

    fn setup_ls(&mut self) {
        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k = random_norm_vector3();
        self.theta = random_angle();
    }

    fn setup_from_str(&mut self, raw: &str) {
        let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

        self.p1 = Vector3::new(data[0], data[1], data[2]);
        self.p2 = Vector3::new(data[6], data[7], data[8]);
        self.k = Vector3::new(data[3], data[4], data[5]);
    }

    fn write_output(&self) -> String {
        format!("{}", self.theta)
    }

    fn run(&mut self) {
        (self.theta, _) = subproblem1(&self.p1, &self.p2, &self.k);
    }

    fn run_report_info(&mut self) -> bool {
        let (theta, is_ls) = subproblem1(&self.p1, &self.p2, &self.k);
        self.theta = theta;
        is_ls
    }

    fn error(&self) -> f64 {
        self.calculate_error(self.theta)
    }

    fn is_at_local_min(&self) -> bool {
        let error = self.error();
        let error_check = error - DELTA;

        self.calculate_error(self.theta + DELTA) >= error_check &&
        self.calculate_error(self.theta - DELTA) >= error_check
    }

    fn name(&self) -> &'static str {
        <Self as SetupStatic>::name()
    }
}

impl SetupDynamic for Subproblem2Setup {
    fn setup(&mut self) {
        let theta1 = random_angle();
        let theta2 = random_angle();

        self.p1 = random_vector3();
        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.theta = SolutionSet2::One((theta1, theta2));

        self.p2 = rot(&self.k2, -theta2) * rot(&self.k1, theta1) * self.p1;
    }

    fn setup_ls(&mut self) {
        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.theta = SolutionSet2::One((random_angle(), random_angle()));
    }

    fn setup_from_str(&mut self, raw: &str) {
        let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

        self.p1 = Vector3::new(data[0], data[1], data[2]);
        self.k1 = Vector3::new(data[3], data[4], data[5]);
        self.k2 = Vector3::new(data[6], data[7], data[8]);
        self.p2 = Vector3::new(data[9], data[10], data[11]);
    }

    fn write_output(&self) -> String {
        format!("{}", self.theta.as_csv())
    }

    fn run(&mut self) {
        (self.theta, _) = subproblem2(&self.p1, &self.p2, &self.k1, &self.k2);
    }

    fn run_report_info(&mut self) -> bool {
        let (theta, is_ls) = subproblem2(&self.p1, &self.p2, &self.k1, &self.k2);
        self.theta = theta;
        is_ls
    }

    fn error(&self) -> f64 {
        let theta = self.theta.get_all();
        let len = theta.len();
        theta.into_iter().map(|t| self.calculate_error(t)).sum::<f64>() / len as f64
    }

    fn is_at_local_min(&self) -> bool {
        for t in self.theta.get_all() {
            for deltas in SolutionSet2::<(f64, f64)>::deltas() {
                let error = self.calculate_error(t);
                let modified_error = self.calculate_error((t.0 + deltas.0, t.1 + deltas.1));

                if error - modified_error > DELTA {
                    return false;
                }
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        <Self as SetupStatic>::name()
    }
}

impl SetupDynamic for Subproblem2ExtendedSetup {
    fn setup(&mut self) {
        self.p0 = random_vector3();
        self.p1 = random_vector3();

        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();

        self.theta1 = random_angle();
        self.theta2 = random_angle();

        self.p2 = rot(&self.k2, -self.theta2) * (self.p0 + rot(&self.k1, self.theta1) * self.p1);
    }

    fn setup_ls(&mut self) {
        unimplemented!();
    }

    fn setup_from_str(&mut self, raw: &str) {
        let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

        self.p0 = Vector3::new(data[0], data[1], data[2]);
        self.p1 = Vector3::new(data[3], data[4], data[5]);
        self.k1 = Vector3::new(data[6], data[7], data[8]);
        self.k2 = Vector3::new(data[9], data[10], data[11]);
        self.p2 = Vector3::new(data[12], data[13], data[14]);
    }

    fn write_output(&self) -> String {
        format!("{},{}", self.theta1, self.theta2)
    }

    fn run(&mut self) {
        (self.theta1, self.theta2) = subproblem2extended(&self.p0, &self.p1, &self.p2, &self.k1, &self.k2);
    }

    fn run_report_info(&mut self) -> bool {
        (self.theta1, self.theta2) = subproblem2extended(&self.p0, &self.p1, &self.p2, &self.k1, &self.k2);
        false
    }

    fn error(&self) -> f64 {
        (self.p0 + rot(&self.k1, self.theta1) * self.p1 - rot(&self.k2, self.theta2) * self.p2).norm()
    }

    fn is_at_local_min(&self) -> bool {
        unimplemented!();
    }

    fn name(&self) -> &'static str {
        <Self as SetupStatic>::name()
    }
}

impl SetupDynamic for Subproblem3Setup {
    fn setup(&mut self) {
        let theta = random_angle();

        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k = random_norm_vector3();
        self.theta = SolutionSet2::One(theta);

        self.d = (self.p2 - rot(&self.k, theta) * self.p1).norm();
    }

    fn setup_ls(&mut self) {
        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k = random_norm_vector3();
        self.d = fastrand::f64();
        self.theta = SolutionSet2::One(random_angle());
    }

    fn setup_from_str(&mut self, raw: &str) {
        let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

        self.p1 = Vector3::new(data[0], data[1], data[2]);
        self.p2 = Vector3::new(data[3], data[4], data[5]);
        self.k = Vector3::new(data[6], data[7], data[8]);
        self.d = data[9];
    }

    fn write_output(&self) -> String {
        format!("{}", self.theta.as_csv())
    }

    fn run(&mut self) {
        (self.theta, _) = subproblem3(&self.p1, &self.p2, &self.k, self.d);
    }

    fn error(&self) -> f64 {
        self.calculate_error(&self.theta.get_all())
    }

    fn run_report_info(&mut self) -> bool {
        let (theta, is_ls) = subproblem3(&self.p1, &self.p2, &self.k, self.d);
        self.theta = theta;
        is_ls
    }

    fn is_at_local_min(&self) -> bool {
        let error = self.error();
        let error_check = error - DELTA;

        let mut solution = self.theta.get_all();

        for i in 0..solution.len() {
            for sign in [-1.0, 1.0] {
                let solution_prev = solution[i];
                solution[i] += sign * DELTA;

                if self.calculate_error(&solution) < error_check {
                    return false;
                }

                solution[i] = solution_prev;
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        <Self as SetupStatic>::name()
    }
}

impl SetupDynamic for Subproblem4Setup {
    fn setup(&mut self) {
        let theta = random_angle();

        self.h = random_norm_vector3();
        self.p = random_vector3();
        self.k = random_norm_vector3();
        self.theta = SolutionSet2::One(theta);

        self.d = (self.h.transpose() * rot(&self.k, theta) * self.p)[0];
    }

    fn setup_ls(&mut self) {
        self.h = random_norm_vector3();
        self.p = random_vector3();
        self.k = random_norm_vector3();
        self.d = fastrand::f64();
        self.theta = SolutionSet2::One(random_angle());
    }

    fn setup_from_str(&mut self, raw: &str) {
        let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

        self.p = Vector3::new(data[0], data[1], data[2]);
        self.k = Vector3::new(data[3], data[4], data[5]);
        self.h = Vector3::new(data[6], data[7], data[8]);
        self.d = data[9];
    }

    fn write_output(&self) -> String {
        format!("{}", self.theta.as_csv())
    }

    fn run(&mut self) {
        (self.theta, _) = subproblem4(&self.h, &self.p, &self.k, self.d);
    }

    fn run_report_info(&mut self) -> bool {
        let (theta, is_ls) = subproblem4(&self.h, &self.p, &self.k, self.d);
        self.theta = theta;
        is_ls
    }

    fn error(&self) -> f64 {
        self.calculate_error(&self.theta.get_all())
    }

    fn is_at_local_min(&self) -> bool {
        let error = self.error();
        let error_check = error - DELTA;

        let mut solution = self.theta.get_all();

        for i in 0..solution.len() {
            for sign in [-1.0, 1.0] {
                let solution_prev = solution[i];
                solution[i] += sign * DELTA;

                if self.calculate_error(&solution) < error_check {
                    return false;
                }

                solution[i] = solution_prev;
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        <Self as SetupStatic>::name()
    }
}

impl SetupDynamic for Subproblem5Setup {
    fn setup(&mut self) {
        let theta1 = random_angle();
        let theta2 = random_angle();
        let theta3 = random_angle();

        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.p3 = random_vector3();

        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.k3 = random_norm_vector3();

        self.theta = SolutionSet4::One((theta1, theta2, theta3));

        self.p0 = -(rot(&self.k1, theta1) * self.p1 - rot(&self.k2, theta2) * (self.p2 + rot(&self.k3, theta3) * self.p3));
    }

    fn setup_ls(&mut self) {
        unimplemented!()
    }

    fn setup_from_str(&mut self, raw: &str) {
        let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

        self.p1 = Vector3::new(data[00], data[01], data[02]);
        self.p2 = Vector3::new(data[03], data[04], data[05]);
        self.p3 = Vector3::new(data[06], data[07], data[08]);
        self.k1 = Vector3::new(data[09], data[10], data[11]);
        self.k2 = Vector3::new(data[12], data[13], data[14]);
        self.k3 = Vector3::new(data[15], data[16], data[17]);
        self.p0 = Vector3::new(data[18], data[19], data[20]);
    }

    fn write_output(&self) -> String {
        format!("{}", self.theta.as_csv())
    }

    fn run(&mut self) {
        self.theta = subproblem5(&self.p0, &self.p1, &self.p2, &self.p3, &self.k1, &self.k2, &self.k3);
    }

    fn run_report_info(&mut self) -> bool {
        self.theta = subproblem5(&self.p0, &self.p1, &self.p2, &self.p3, &self.k1, &self.k2, &self.k3);
        false
    }

    fn error(&self) -> f64 {
        let theta = self.theta.get_all();
        let len = theta.len();

        if len == 0 {
            return 0.0;
        }

        theta
            .into_iter()
            .map(|(t1, t2, t3)| {
                (self.p0 + rot(&self.k1, t1) * self.p1 - rot(&self.k2, t2) * (self.p2 + rot(&self.k3, t3) * self.p3)).norm()
            })
            .sum::<f64>() / len as f64
    }

    fn is_at_local_min(&self) -> bool {
        unimplemented!()
    }

    fn name(&self) -> &'static str {
        <Self as SetupStatic>::name()
    }
}

impl SetupDynamic for Subproblem6Setup {
    fn setup(&mut self) {
        let theta1 = random_angle();
        let theta2 = random_angle();

        self.h = [Vector3::zeros(); 4];
        self.k = [Vector3::zeros(); 4];
        self.p = [Vector3::zeros(); 4];

        for (h, (k, p)) in self.h.iter_mut().zip(self.k.iter_mut().zip(self.p.iter_mut())) {
            *h = random_norm_vector3();
            *k = random_norm_vector3();
            *p = random_vector3();
        }

        self.d1 = (
            self.h[0].transpose() * rot(&self.k[0], theta1) * self.p[0] +
            self.h[1].transpose() * rot(&self.k[1], theta2) * self.p[1]
        )[0];

        self.d2 = (
            self.h[2].transpose() * rot(&self.k[2], theta1) * self.p[2] +
            self.h[3].transpose() * rot(&self.k[3], theta2) * self.p[3]
        )[0];

        self.theta = SolutionSet4::One((theta1, theta2));
    }

    fn setup_ls(&mut self) {
        unimplemented!()
    }

    fn setup_from_str(&mut self, raw: &str) {
        let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();
        let mut i = 0;

        for v in self.h.iter_mut() {
            *v = Vector3::new(data[i], data[i + 1], data[i + 2]);
            i += 3;
        }

        for v in self.k.iter_mut() {
            *v = Vector3::new(data[i], data[i + 1], data[i + 2]);
            i += 3;
        }

        for v in self.p.iter_mut() {
            *v = Vector3::new(data[i], data[i + 1], data[i + 2]);
            i += 3;
        }

        self.d1 = data[i];
        self.d2 = data[i + 1];
    }

    fn write_output(&self) -> String {
        format!("{}", self.theta.as_csv())
    }

    fn run(&mut self) {
        self.theta = subproblem6(
            &self.h,
            &self.k,
            &self.p,
            self.d1,
            self.d2,
        );
    }

    fn run_report_info(&mut self) -> bool {
        self.theta = subproblem6(
            &self.h,
            &self.k,
            &self.p,
            self.d1,
            self.d2,
        );
        false
    }

    fn error(&self) -> f64 {
        let theta = self.theta.get_all();
        let len = theta.len();

        theta.into_iter().map(|(t1, t2)| {
            (Vector2::new(
                (self.h[0].transpose() * rot(&self.k[0], t1) * self.p[0] + self.h[1].transpose() * rot(&self.k[1], t2) * self.p[1])[0] - self.d1,
                (self.h[2].transpose() * rot(&self.k[2], t1) * self.p[2] + self.h[3].transpose() * rot(&self.k[3], t2) * self.p[3])[0] - self.d2
            )).norm()
        }).sum::<f64>() / len as f64
    }

    fn is_at_local_min(&self) -> bool {
        unimplemented!()
    }

    fn name(&self) -> &'static str {
        <Self as SetupStatic>::name()
    }
}
