use std::f64::consts::{PI, TAU};

pub mod auxiliary;
pub mod setups;
pub mod hardcoded;

use {
    nalgebra::{ Vector3, Vector6, Matrix3 },

    crate::subproblems::{
        subproblem1,
        subproblem2,
        subproblem3,
        subproblem4,
        subproblem5,

        auxiliary::rot
    },

    auxiliary::Kinematics,
};

pub fn spherical_two_parallel(r_0t: &Matrix3<f64>, p_0t: &Vector3<f64>, kin: &Kinematics) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let mut q = Vec::with_capacity(6);
    let mut is_ls = Vec::with_capacity(6);

    let (t1, q1_is_ls) = subproblem4(
        &(kin.h.column(1).into()),
        &(p_0t - r_0t * kin.p.column(6) - kin.p.column(0)),
        &(-kin.h.column(0)),
        (kin.h.column(1).transpose() * (kin.p.column(1) + kin.p.column(2) + kin.p.column(3)))[0],
    );

    for q1 in t1.get_all() {
        let (t3, q3_is_ls) = subproblem3(
            &(-kin.p.column(3)),
            &(kin.p.column(2).into()),
            &(kin.h.column(2).into()),
            (rot(&(-kin.h.column(0)), q1) * (-p_0t + r_0t * kin.p.column(6) + kin.p.column(0)) + kin.p.column(1)).norm(),
        );

        for q3 in t3.get_all() {
            let (q2, q2_is_ls) = subproblem1(
                &(-kin.p.column(2) - rot(&kin.h.column(2).into(), q3) * kin.p.column(3)),
                &(rot(&(-kin.h.column(0)), q1) * (-p_0t + r_0t * kin.p.column(6) + kin.p.column(0)) + kin.p.column(1)),
                &(kin.h.column(1).into()),
            );

            let r_36 = rot(&(-kin.h.column(2)), q3) *
                rot(&(-kin.h.column(1)), q2) *
                rot(&(-kin.h.column(0)), q1) * r_0t;

            let (t5, q5_is_ls) = subproblem4(
                &(kin.h.column(3).into()),
                &(kin.h.column(5).into()),
                &(kin.h.column(4).into()),
                (kin.h.column(3).transpose() * r_36 * kin.h.column(5))[0],
            );

            for q5 in t5.get_all() {
                let (q4, q4_is_ls) = subproblem1(
                    &(rot(&(kin.h.column(4).into()), q5) * kin.h.column(5)),
                    &(r_36 * kin.h.column(5)),
                    &(kin.h.column(3).into()),
                );

                let (q6, q6_is_ls) = subproblem1(
                    &(rot(&(-kin.h.column(4)), q5) * kin.h.column(3)),
                    &(r_36.transpose() * kin.h.column(3)),
                    &(-kin.h.column(5)),
                );

                q.push(Vector6::new(q1, q2, q3, q4, q5, q6));
                is_ls.push(q1_is_ls || q2_is_ls || q3_is_ls || q4_is_ls || q5_is_ls || q6_is_ls)
            }
        }
    }

    (q, is_ls)
}

pub fn spherical_two_intersecting(r_0t: &Matrix3<f64>, p_0t: &Vector3<f64>, kin: &Kinematics) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let mut q = Vec::with_capacity(6);
    let mut is_ls = Vec::with_capacity(6);

    let p_16 = p_0t - r_0t * kin.p.column(6) - kin.p.column(0);

    let (t3, t3_is_ls) = subproblem3(&kin.p.column(3).into(), &-kin.p.column(2), &kin.h.column(2).into(), p_16.norm());

    for q3 in t3.get_all() {
        let (t12, t12_is_ls) = subproblem2(
            &p_16,
            &(kin.p.column(2) + rot(&kin.h.column(2).into(), q3) * kin.p.column(3)),
            &-kin.h.column(0),
            &kin.h.column(1).into()
        );

        for (q1, q2) in t12.get_all() {
            let r_36 =
                rot(&-kin.h.column(2), q3) *
                rot(&-kin.h.column(1), q2) *
                rot(&-kin.h.column(0), q1) *
                r_0t;

            let (t5, q5_is_ls) = subproblem4(
                &kin.h.column(3).into(),
                &kin.h.column(5).into(),
                &kin.h.column(4).into(),
                (kin.h.column(3).transpose() * r_36 * kin.h.column(5))[0],
            );

            for q5 in t5.get_all() {
                let (q4, q4_is_ls) = subproblem1(
                    &(rot(&kin.h.column(4).into(), q5) * kin.h.column(5)),
                    &(r_36 * kin.h.column(5)),
                    &kin.h.column(3).into(),
                );

                let (q6, q6_is_ls) = subproblem1(
                    &(rot(&-kin.h.column(4), q5) * kin.h.column(3)),
                    &(r_36.transpose() * kin.h.column(3)),
                    &-kin.h.column(5),
                );

                q.push(Vector6::new(q1, q2, q3, q4, q5, q6));
                is_ls.push(t3_is_ls || t12_is_ls || q5_is_ls || q4_is_ls || q6_is_ls);
            }
        }
    }

    (q, is_ls)
}

pub fn spherical(r_06: &Matrix3<f64>, p_0t: &Vector3<f64>, kin: &Kinematics) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let mut q = Vec::with_capacity(6);
    let mut is_ls = Vec::with_capacity(6);

    let p_16 = p_0t - kin.p.column(0) - r_06 * kin.p.column(6);

    let t123 = subproblem5(
        &-kin.p.column(1),
        &p_16,
        &kin.p.column(2).into(),
        &kin.p.column(3).into(),
        &-kin.h.column(0),
        &kin.h.column(1).into(),
        &kin.h.column(2).into(),
    );

    for (q1, q2, q3) in t123.get_all() {
        let r_36 = rot(&-kin.h.column(2), q3) *
            rot(&-kin.h.column(1), q2) *
            rot(&-kin.h.column(0), q1) * r_06;

        let (t5, q5_is_ls) = subproblem4(
            &kin.h.column(3).into(),
            &kin.h.column(5).into(),
            &kin.h.column(4).into(),
            (kin.h.column(3).transpose() * r_36 * kin.h.column(5))[0],
        );

        for q5 in t5.get_all() {
            let (q4, q4_is_ls) = subproblem1(
                &(rot(&kin.h.column(4).into(), q5) * kin.h.column(5)),
                &(r_36 * kin.h.column(5)),
                &kin.h.column(3).into(),
            );

            let (q6, q6_is_ls) = subproblem1(
                &(rot(&-kin.h.column(4), q5) * kin.h.column(3)),
                &(r_36.transpose() * kin.h.column(3)),
                &-kin.h.column(5)
            );

            q.push(Vector6::new(q1, q2, q3, q4, q5, q6));
            is_ls.push(q5_is_ls || q4_is_ls || q6_is_ls);
        }
    }

    (q, is_ls)
}

pub fn three_parallel_two_intersecting(r_06: &Matrix3<f64>, p_0t: &Vector3<f64>, kin: &Kinematics) -> (Vec<Vector6<f64>>, Vec<bool>) {
    let mut q = Vec::with_capacity(6);
    let mut is_ls = Vec::with_capacity(6);

    let sum_p_2_5 = kin.p.slice((0, 1), (3, 4)).column_sum();
    let p_16 = p_0t - kin.p.column(0) - r_06 * kin.p.column(6);

    let (theta1, theta1_is_ls) = subproblem4(&kin.h.column(1).into(), &p_16, &-kin.h.column(0), (kin.h.column(1).transpose() * sum_p_2_5)[0]);

    for q1 in theta1.get_all() {
        let r_01 = rot(&kin.h.column(0).into(), q1);

        let (theta5, theta5_is_ls) = subproblem4(
            &kin.h.column(1).into(),
            &kin.h.column(5).into(),
            &kin.h.column(4).into(),
            (kin.h.column(1).transpose() * r_01.transpose() * r_06 * kin.h.column(5))[0],
        );

        for q5 in theta5.get_all() {
            let (theta_14, theta_14_is_ls) = subproblem1(
                &(rot(&kin.h.column(4).into(), q5) * kin.h.column(5)),
                &(r_01.transpose() * r_06 * kin.h.column(5)),
                &kin.h.column(1).into()
            );

            let r_01 = rot(&kin.h.column(0).into(), q1);
            let r_45 = rot(&kin.h.column(4).into(), q5);
            let r_14 = rot(&kin.h.column(1).into(), theta_14);

            let p_12: &Vector3<f64> = &kin.p.column(1).into();
            let p_23: &Vector3<f64> = &kin.p.column(2).into();
            let p_34: &Vector3<f64> = &kin.p.column(3).into();
            let p_45: &Vector3<f64> = &(kin.p.column(4) + kin.p.column(5));

            let d_inner = r_01.transpose() * p_16 - p_12 - r_14 * p_45;
            let d = d_inner.norm();

            let (theta_3, theta_3_is_ls) = subproblem3(&-p_34, p_23, &kin.h.column(1).into(), d);

            for q3 in theta_3.get_all() {
                let (q2, q2_is_ls) = subproblem1(&(p_23 + rot(&kin.h.column(1).into(), q3) * p_34), &d_inner, &kin.h.column(1).into());

                let q4 = (theta_14 - q2 - q3 + PI) % TAU - PI;

                let (q6, q6_is_ls) = subproblem1(
                    &kin.h.column(4).into(),
                    &(r_45.transpose() * r_14.transpose() * r_01.transpose() * r_06 * kin.h.column(4)),
                    &kin.h.column(5).into(),
                );

                q.push(Vector6::new(q1, q2, q3, q4, q5, q6));
                is_ls.push(theta1_is_ls || theta5_is_ls || theta_14_is_ls || theta_3_is_ls || q2_is_ls || q6_is_ls);
            }
        }
    }

    (q, is_ls)
}