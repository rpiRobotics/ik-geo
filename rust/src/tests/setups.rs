use {
    crate::{
        tests::auxiliary::{
            random_vector3,
            random_angle,
            rot,
            random_norm_vector3
        },

        subproblems::{ SolutionSet2, subproblem1, subproblem2, subproblem2extended, subproblem3 }
    },

    nalgebra::Vector3,
};

const DELTA: f64 = 1e-12;

/// An interface for setting up subproblem testing. Resposible for generating parameters, running
/// the function, and calculating data such as the error.
pub trait Setup {
    /// Initialize parameters to test the case where theta is solved for exactly
    fn setup(&mut self);

    /// Initialize parameters to test the case where theta is minimized using least squares
    fn setup_ls(&mut self);

    fn run(&mut self);

    fn error(&self) -> f64;
    fn is_at_local_min(&self) -> bool;
    fn name(&self) -> &'static str;
}

pub struct Subproblem1Setup {
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    k: Vector3<f64>,

    theta: f64,
}

#[derive(Clone)]
pub struct Subproblem2Setup {
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    k1: Vector3<f64>,
    k2: Vector3<f64>,

    theta1: SolutionSet2<f64>,
    theta2: SolutionSet2<f64>,
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

impl Subproblem1Setup {
    pub fn new() -> Self {
        Self {
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            k: Vector3::zeros(),

            theta: 0.0,
        }
    }

    fn calculate_error(&self, theta: f64) -> f64 {
        (self.p2 - rot(self.k, theta) * self.p1).norm()
    }
}

impl Subproblem2Setup {
    pub fn new() -> Self {
        Self {
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            k1: Vector3::zeros(),
            k2: Vector3::zeros(),

            theta1: SolutionSet2::One(0.0),
            theta2: SolutionSet2::One(0.0),
        }
    }

    fn calculate_error(&self, theta1: &[f64], theta2: &[f64]) -> f64 {
        theta1.iter()
            .zip(theta2.iter())
            .map(|(&t1, &t2)| {
                (rot(self.k2, t2) * self.p2 - rot(self.k1,t1) * self.p1).norm()
            })
            .sum()
    }
}

impl Subproblem2ExtendedSetup {
    pub fn new() -> Self {
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
}

impl Subproblem3Setup {
    pub fn new() -> Self {
        Self {
            p1: Vector3::zeros(),
            p2: Vector3::zeros(),
            k: Vector3::zeros(),
            d: 0.0,

            theta: SolutionSet2::One(0.0),
        }
    }

    pub fn calculate_error(&self, theta: &[f64]) -> f64 {
        theta
            .iter()
            .map(|&t| {
                ((self.p2 - rot(self.k, t) * self.p1).norm() - self.d).abs()
            })
            .sum()
    }
}

impl Setup for Subproblem1Setup {
    fn setup(&mut self) {
        self.p1 = random_vector3();
        self.k = random_norm_vector3();
        self.theta = random_angle();

        self.p2 = rot(self.k, self.theta) * self.p1;
    }

    fn error(&self) -> f64 {
        self.calculate_error(self.theta)
    }

    fn setup_ls(&mut self) {
        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k = random_norm_vector3();
        self.theta = random_angle();
    }

    fn is_at_local_min(&self) -> bool {
        let error = self.error();
        let error_check = error - DELTA;

        self.calculate_error(self.theta + DELTA) >= error_check &&
        self.calculate_error(self.theta - DELTA) >= error_check
    }

    fn run(&mut self) {
        (self.theta, _) = subproblem1(&self.p1, &self.p2, &self.k);
    }

    fn name(&self) -> &'static str {
        "Subproblem 1"
    }
}

impl Setup for Subproblem2Setup {
    fn setup(&mut self) {
        let theta1 = random_angle();
        let theta2 = random_angle();

        self.p1 = random_vector3();
        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.theta1 = SolutionSet2::One(theta1);
        self.theta2 = SolutionSet2::One(theta2);

        self.p2 = rot(self.k2, -theta2) * rot(self.k1, theta1) * self.p1;
    }

    fn setup_ls(&mut self) {
        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.theta1 = SolutionSet2::One(random_angle());
        self.theta2 = SolutionSet2::One(random_angle());
    }

    fn run(&mut self) {
        (self.theta1, self.theta2, _) = subproblem2(&self.p1, &self.p2, &self.k1, &self.k2);
    }

    fn error(&self) -> f64 {
        self.calculate_error(&self.theta1.get_all(), &self.theta2.get_all())
    }

    fn is_at_local_min(&self) -> bool {
        let error = self.error();
        let error_check = error - DELTA;

        let mut solutions = [self.theta1.get_all(), self.theta2.get_all()];

        for i in 0..solutions.len() {
            for j in 0..solutions[i].len() {
                for sign in [-1.0, 1.0] {
                    let solution_prev = solutions[i][j];
                    solutions[i][j] += sign * DELTA;

                    if self.calculate_error(&solutions[0], &solutions[1]) < error_check {
                        return false;
                    }

                    solutions[i][j] = solution_prev;
                }
            }
        }

        true
    }

    fn name(&self) -> &'static str {
        "Subproblem 2"
    }
}

impl Setup for Subproblem2ExtendedSetup {
    fn setup(&mut self) {
        self.p0 = random_vector3();
        self.p1 = random_vector3();

        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();

        self.theta1 = random_angle();
        self.theta2 = random_angle();

        self.p2 = rot(self.k2, -self.theta2) * (self.p0 + rot(self.k1, self.theta1) * self.p1);
    }

    fn setup_ls(&mut self) {
        unimplemented!();
    }

    fn run(&mut self) {
        (self.theta1, self.theta2) = subproblem2extended(&self.p0, &self.p1, &self.p2, &self.k1, &self.k2);
    }

    fn error(&self) -> f64 {
        (self.p0 + rot(self.k1, self.theta1) * self.p1 - rot(self.k2, self.theta2) * self.p2).norm()
    }

    fn is_at_local_min(&self) -> bool {
        unimplemented!();
    }

    fn name(&self) -> &'static str {
        "Subproblem 2 Ex"
    }
}

impl Setup for Subproblem3Setup {
    fn setup(&mut self) {
        let theta = random_angle();

        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k = random_norm_vector3();
        self.theta = SolutionSet2::One(theta);

        self.d = (self.p2 - rot(self.k, theta) * self.p1).norm();
    }

    fn setup_ls(&mut self) {
        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k = random_norm_vector3();
        self.d = fastrand::f64();
        self.theta = SolutionSet2::One(random_angle());
    }

    fn run(&mut self) {
        (self.theta, _) = subproblem3(&self.p1, &self.p2, &self.k, self.d);
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
        "Subproblem 3"
    }
}
