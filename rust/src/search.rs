use crate::inverse_kinematics::auxiliary::{search_1d, Vector};

#[test]
fn search() {
    println!("{:?}", search_1d(|x| Vector::<f64, 4>::new(
        x,
        x * (x - 1.0),
        x * (x - 1.0) * (x - 2.0),
        x * (x - 1.0) * (x - 2.0) * (x - 3.0)
    ), -1.0, 5.0, 1000));
}
