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
