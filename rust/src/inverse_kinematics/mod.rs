pub mod auxiliary;
pub mod setups;

use {
    nalgebra::{ Vector3, Vector6, Matrix3 },

    crate::subproblems::{
        subproblem1,
        subproblem3,
        subproblem4,

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
