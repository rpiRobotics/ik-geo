use nalgebra::{Vector1, Vector2};

use crate::inverse_kinematics::auxiliary::search_2d;

#[test]
fn search() {
    let result = search_2d(|x, y| Vector1::new(
        (3.0 * x.hypot(y)).sin().powi(2),
    ), (-3.0, -3.0), (3.0, 3.0), 100);

    println!("{result:?}");
}
