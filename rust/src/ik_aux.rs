use std::fmt::{Display, Formatter};

use nalgebra::{Vector6, U7, U3, U8, Matrix3, Vector3, Matrix, ArrayStorage};

use crate::auxiliary::rot;

pub type Matrix3x7<T> = Matrix<T, U3, U7, ArrayStorage<T, 3, 7>>;
pub type Matrix3x8<T> = Matrix<T, U3, U8, ArrayStorage<T, 3, 8>>;

#[derive(Debug)]
pub struct Kinematics {
    pub joint_type: Vector6<u8>,
    pub h: Matrix3x7<f64>,
    pub p: Matrix3x8<f64>,
}

impl Kinematics {
    pub fn new() -> Self {
        Self {
            joint_type: Vector6::zeros(),
            h: Matrix3x7::zeros(),
            p: Matrix3x8::zeros(),
        }
    }
}

impl Display for Kinematics {
    fn fmt(&self, f: &mut Formatter) -> std::fmt::Result {
        write!(f, "\njoint type:{}h:{}p:{}", self.joint_type, self.h, self.p)
    }
}

pub fn forward_kinematics(kin: &Kinematics, theta: &Vector6<f64>) -> (Matrix3<f64>, Vector3<f64>) {
    let mut p: Vector3<f64> = kin.p.column(0).into();
    let mut r = Matrix3::identity();

    for (i, (&jt, &t)) in kin.joint_type.iter().zip(theta.iter()).enumerate() {
        if jt == 0 || jt == 2 {
            r = r * rot(&kin.h.column(i).into(), t);
        }
        else if jt == 1 || jt == 3 {
            p = p + r * kin.h.column(i) * t;
        }

        p = p + r * kin.p.column(i + 1);
    }

    (r, p)
}
