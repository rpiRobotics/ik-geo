use {
    std::fmt::{ Display, Formatter },

    nalgebra::{
        U3, U7, U8,
        Vector3,
        Matrix, Matrix3,
        ArrayStorage,
    },

    crate::subproblems::auxiliary::rot,
};

pub type Matrix3x7<T> = Matrix<T, U3, U7, ArrayStorage<T, 3, 7>>;
pub type Matrix3x8<T> = Matrix<T, U3, U8, ArrayStorage<T, 3, 8>>;

#[derive(Debug, Clone)]
pub struct Kinematics {
    pub h: Matrix3x7<f64>,
    pub p: Matrix3x7<f64>,
}

impl Kinematics {
    pub fn new() -> Self {
        Self {
            h: Matrix3x7::zeros(),
            p: Matrix3x7::zeros(),
        }
    }
}

impl Display for Kinematics {
    fn fmt(&self, f: &mut Formatter) -> std::fmt::Result {
        write!(f, "\nh:{}p:{}", self.h, self.p)
    }
}

pub fn forward_kinematics(kin: &Kinematics, theta: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
    let mut p: Vector3<f64> = kin.p.column(0).into();
    let mut r = Matrix3::identity();

    for (i, &t) in theta.iter().enumerate() {
        // p = p + r * kin.h.column(i) * t;

        r = r * rot(&kin.h.column(i).into(), t);
        p = p + r * kin.p.column(i + 1);
    }

    (r, p)
}

pub fn forward_kinematics_general(kin: &Kinematics, theta: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
    let mut p: Vector3<f64> = kin.p.column(0).into();
    let mut r = Matrix3::identity();

    for (i, t) in theta.iter().enumerate() {
        r = r * rot(&kin.h.column(i).into(), *t);
        p = p + r * kin.p.column(i + 1);
    }

    (r, p)
}

pub fn forward_kinematics_partial(kin: &Kinematics, q_n: f64, n: usize, r_6t: &Matrix3<f64>) -> (Kinematics, Matrix3<f64>) {
    let mut kin_new = kin.clone();
    let r_n = rot(&kin.h.column(n).into(), q_n);

    for i in 0..kin_new.h.ncols() {
        if i > n {
            kin_new.h.set_column(i - 1, &(r_n * kin_new.h.column(i)));
        }
    }

    for i in 0..kin_new.p.ncols() {
        if i == n + 1 {
            kin_new.p.set_column(i - 1, &(kin_new.p.column(i - 1) + r_n * kin_new.p.column(i)));

        }
        else if i > n + 2 {
            kin_new.p.set_column(i - 1, &(r_n * kin_new.p.column(i)));
        }
    }

    (kin_new, r_n * r_6t)
}
