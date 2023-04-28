use nalgebra::{Vector1, Vector2, Vector6, Vector3, Matrix3x6, Matrix3};

use crate::{inverse_kinematics::{auxiliary::{search_2d, Matrix3x7, Kinematics}, gen_six_dof}, subproblems::Vector7};

#[test]
fn search() {
    let h = Matrix3x6::from_columns(&[
        Vector3::new(0.4959, 0.6393, -0.5877),
        Vector3::new(0.6984, 0.2236, -0.6799),
        Vector3::new(-0.4339, 0.0918, 0.8963),
        Vector3::new(0.6241, -0.4597, 0.6318),
        Vector3::new(0.8355, -0.0267, 0.5488),
        Vector3::new(-0.6461, -0.1412, 0.7501),
    ]);

    let p = Matrix3x7::from_columns(&[
        Vector3::new(0.3897, -0.3658, 0.9004),
        Vector3::new(-0.9311, -0.1225, -0.2369),
        Vector3::new(0.5310, 0.5904, -0.6263),
        Vector3::new(-0.0205, -0.1088, 0.2926),
        Vector3::new(0.4187, 0.5094, -0.4479),
        Vector3::new(0.3594, 0.3102, -0.6748),
        Vector3::new(-0.7620, -0.0033, 0.9195),
    ]);

    let r = Matrix3::new(
        -0.1656, 0.5319, -0.8304,
        -0.9852, -0.1276, 0.1147,
        -0.0450, 0.8371, 0.5452,
    );

    let t = Vector3::new(
        0.2476,
        0.0045,
        1.2865,
    );

    println!("h{h}p{p}r{r}t{t}");

    let (q, is_ls) = gen_six_dof(&r, &t, &Kinematics { h, p });

    println!("len q = {}", q.len());

    for (i, (q_i, ls)) in q.into_iter().zip(is_ls.into_iter()).enumerate() {
        println!("{i}{ls}{q_i}");
    }
}
