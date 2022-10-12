use {
    crate::{
        tests::auxiliary::{
            random_vector3,
            random_angle,
            rot,
            random_norm_vector3
        },

        subproblems::{ subproblem1, subproblem2 }
    },

    nalgebra::{ Vector3, DVector },
};

/// An interface for setting up subproblem testing. Resposible for generating parameters, running
/// the function, and calculating data such as the error.
pub trait Setup {
    fn name(&self) -> &'static str;
    fn is_at_local_min(&self) -> bool;
    fn error(&self) -> f64;

    /// Initialize parameters to test the case where theta is solved for exactly
    fn setup(&mut self);

    /// Initialize parameters to test the case where theta is minimized using least squares
    fn setup_ls(&mut self);

    fn run(&mut self);
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

    theta1: DVector<f64>,
    theta2: DVector<f64>,
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

            theta1: DVector::zeros(0),
            theta2: DVector::zeros(0),
        }
    }

    fn calculate_error(&self, theta1: &DVector<f64>, theta2: &DVector<f64>) -> f64 {
        theta1.iter()
            .zip(theta2.iter())
            .map(|(&t1, &t2)| {
                (rot(self.k2, t2) * self.p2 - rot(self.k1,t1) * self.p1).norm()
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
        const DELTA: f64 = 1e-12;

        let error = self.error();
        let error_check = error - DELTA;

        self.calculate_error(self.theta + DELTA) >= error_check &&
        self.calculate_error(self.theta - DELTA) >= error_check
    }

    fn run(&mut self) {
        (self.theta, _) = subproblem1(self.p1, self.p2, self.k);
    }

    fn name(&self) -> &'static str {
        "Subproblem 1"
    }
}

impl Setup for Subproblem2Setup {
    fn setup(&mut self) {
        self.p1 = random_vector3();
        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.theta1 = DVector::from_vec(vec![random_angle()]);
        self.theta2 = DVector::from_vec(vec![random_angle()]);

        self.p2 = rot(self.k2, -self.theta2[0]) * rot(self.k1,self.theta1[0]) * self.p1;
    }

    fn setup_ls(&mut self) {
        self.p1 = random_vector3();
        self.p2 = random_vector3();
        self.k1 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.k2 = random_norm_vector3();
        self.theta1 = DVector::from_vec(vec![random_angle()]);
        self.theta2 = DVector::from_vec(vec![random_angle()]);
    }

    fn run(&mut self) {
        (self.theta1, self.theta2, _) = subproblem2(self.p1, self.p2, self.k1, self.k2);
    }

    fn error(&self) -> f64 {
        self.calculate_error(&self.theta1, &self.theta2)
    }

    fn is_at_local_min(&self) -> bool {
        const DELTA: f64 = 1e-12;

        let error = self.error();
        let error_check = error - DELTA;

        let mut solutions = [self.theta1.clone(), self.theta2.clone()];

        for i in 0..solutions.len() {
            for j in 0..self.theta1.len() {
                for sign in [-1.0, 1.0] {
                    let solution_prev = solutions[i][j];
                    solutions[i][j] += sign;

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
