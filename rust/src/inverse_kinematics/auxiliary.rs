use {
    nalgebra::{
        Const,
        U1, U3, U7, U8,
        Vector3,
        Matrix, Matrix3,
        ArrayStorage,
    },

    std::f64::consts::{ PI, TAU },

    crate::subproblems::auxiliary::rot,
};

pub type Matrix3x7<T> = Matrix<T, U3, U7, ArrayStorage<T, 3, 7>>;
pub type Matrix3x8<T> = Matrix<T, U3, U8, ArrayStorage<T, 3, 8>>;

pub type Vector<T, const N: usize> = Matrix<T, Const<N>, U1, ArrayStorage<f64, N, 1>>;

#[derive(Debug, Clone)]
pub struct Kinematics<const C1: usize, const C2: usize> { // TODO: somehow statically ensure that C2 - C1 = 1
    pub h: Matrix<f64, U3, Const<C1>, ArrayStorage<f64, 3, C1>>,
    pub p: Matrix<f64, U3, Const<C2>, ArrayStorage<f64, 3, C2>>,
}

impl<const C1: usize, const C2: usize> Kinematics<C1, C2> {
    pub fn new() -> Self {
        Self {
            h: Matrix::<f64, U3, Const<C1>, ArrayStorage<f64, 3, C1>>::zeros(),
            p: Matrix::<f64, U3, Const<C2>, ArrayStorage<f64, 3, C2>>::zeros(),
        }
    }

    pub fn forward_kinematics(&self, theta: &Matrix<f64, Const<C1>, U1, ArrayStorage<f64, C1, 1>>) -> (Matrix3<f64>, Vector3<f64>) {
        let mut p: Vector3<f64> = self.p.column(0).into();
        let mut r = Matrix3::identity();

        for (i, &t) in theta.iter().enumerate() {
            r = r * rot(&self.h.column(i).into(), t);
            p = p + r * self.p.column(i + 1);
        }

        (r, p)
    }
}

impl Kinematics<7, 8> {
    pub fn forward_kinematics_partial(&self, q_n: f64, n: usize, r_6t: &Matrix3<f64>) -> (Kinematics<6, 7>, Matrix3<f64>) {
        let mut kin_new: Kinematics<6, 7> = Kinematics::new();
        let r_n = rot(&self.h.column(n).into(), q_n);

        for i in 0..self.h.ncols() {
            if i > n {
                kin_new.h.set_column(i - 1, &(r_n * self.h.column(i)));
            }
            else {
                kin_new.h.set_column(i, &self.h.column(i));
            }
        }

        for i in 0..self.p.ncols() {
            if i == n {
                kin_new.p.set_column(i, &(self.p.column(i) + r_n * self.p.column(i + 1)));

            }
            else if i > n + 1 {
                kin_new.p.set_column(i - 1, &(r_n * self.p.column(i)));
            }
            else {
                kin_new.p.set_column(i, &self.p.column(i));
            }
        }

        (kin_new, r_n * r_6t)
    }
}

pub fn wrap_to_pi(theta: f64) -> f64 {
    (theta + PI).rem_euclid(TAU) - PI
}

fn find_zero<const N: usize, F: Fn(f64) -> Vector<f64, N>>(f: F, left: f64, right: f64, i: usize) -> Option<f64> {
    const ITERATIONS: usize = 100;
    const EPSILON: f64 = 1e-5;

    let mut x_left = left;
    let mut x_right = right;

    let mut y_left = f(x_left)[i];
    let mut y_right = f(x_right)[i];

    for _ in 0..ITERATIONS {
        let delta = y_right - y_left;

        if delta.abs() < EPSILON {
            break;
        }

        let x_0 = x_left - y_left * (x_right - x_left) / delta;
        let y_0 = f(x_0)[i];

        if !y_0.is_finite() {
            return None;
        }

        if (y_left < 0.0) != (y_0 < 0.0) {
            x_left = x_0;
            y_left = y_0;
        }
        else {
            x_right = x_0;
            y_right = y_0;
        }
    }

    if left <= x_left && x_left <= right {
        Some(x_left)
    }
    else {
        None
    }
}

pub fn search_1d<const N: usize, F: Fn(f64) -> Vector<f64, N>>(f: F, left: f64, right: f64, initial_samples: usize) -> Vec<(f64, usize)> {
    const CROSS_THRESHOLD: f64 = 0.1;

    let delta = (right - left) / initial_samples as f64;

    let mut last_v = f(left);
    let mut x = left + delta;

    let mut zeros = Vec::with_capacity(8);

    for _ in 0..initial_samples {
        let v = f(x);

        for (i, (&y, &last_y)) in v.iter().zip(last_v.into_iter()).enumerate() {
            if (y < 0.0) != (last_y < 0.0) && y.abs() < CROSS_THRESHOLD && last_y.abs() < CROSS_THRESHOLD {
                if let Some(z) = find_zero(&f, x - delta, x, i) {
                    zeros.push((z, i));
                }
            }
        }

        last_v = v;
        x += delta;
    }

    zeros
}
