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

#[derive(Debug, Clone)]
pub struct Kinematics3x8 {
    pub h: Matrix3x7<f64>,
    pub p: Matrix3x8<f64>,
}

impl Kinematics {
    pub fn new() -> Self {
        Self {
            h: Matrix3x7::zeros(),
            p: Matrix3x7::zeros(),
        }
    }
}

impl Kinematics3x8 {
    pub fn new() -> Self {
        Self {
            h: Matrix3x7::zeros(),
            p: Matrix3x8::zeros(),
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

pub fn forward_kinematics3x8(kin: &Kinematics3x8, theta: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
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

pub fn forward_kinematics_general3x8(kin: &Kinematics3x8, theta: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
    let mut p: Vector3<f64> = kin.p.column(0).into();
    let mut r = Matrix3::identity();

    for (i, t) in theta.iter().enumerate() {
        r = r * rot(&kin.h.column(i).into(), *t);
        p = p + r * kin.p.column(i + 1);
    }

    (r, p)
}

pub fn forward_kinematics_partial(kin: &Kinematics3x8, q_n: f64, n: usize, r_6t: &Matrix3<f64>) -> (Kinematics, Matrix3<f64>) {
    let mut kin_new = Kinematics::new();
    let r_n = rot(&kin.h.column(n).into(), q_n);

    for i in 0..kin.h.ncols() {
        if i > n {
            kin_new.h.set_column(i - 1, &(r_n * kin.h.column(i)));
        }
        else {
            kin_new.h.set_column(i, &kin.h.column(i));
        }
    }

    for i in 0..kin.p.ncols() {
        if i == n {
            kin_new.p.set_column(i, &(kin.p.column(i) + r_n * kin.p.column(i + 1)));

        }
        else if i > n + 1 {
            kin_new.p.set_column(i - 1, &(r_n * kin.p.column(i)));
        }
        else {
            kin_new.p.set_column(i, &kin.p.column(i));
        }
    }

    (kin_new, r_n * r_6t)
}