pub use nalgebra;

mod auxiliary;
mod ik_aux;

pub mod solutionset;
pub mod subproblems;
pub mod setups;

pub mod inverse_kinematics;
pub mod ik_setups;


#[cfg(test)]
mod correctness;
