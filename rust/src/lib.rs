pub use nalgebra;

pub mod solutionset;

pub mod subproblems;
pub mod inverse_kinematics;

#[cfg(test)]
mod correctness;

#[cfg(test)]
mod timing;
