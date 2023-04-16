use std::f64::NAN;

use crate::inverse_kinematics::{setups::{calculate_ik_error, ik_write_output}, hardcoded::setups::hardcoded_setup_from_string};

use {
    crate::{
        inverse_kinematics::{
            auxiliary::{ Kinematics, Matrix3x7 },
            setups::SetupIk
        },

        subproblems::auxiliary::random_angle
    },

    std::fs,

    nalgebra::{ Matrix3, Vector3, Vector6, Matrix3x6 },
};

const CONFIG_PATH: &'static str = "lib_ikfast.config";

pub struct IkFast {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
}

pub struct IkFastConfig {
    kin: Kinematics<6, 7>,
}

impl IkFastConfig {
    pub fn load(path: &str) -> Result<Self, String> {
        let raw = fs::read_to_string(path).expect(&format!("Failed to read from {path}"));

        let mut this = IkFastConfig {
            kin: Kinematics { h: Matrix3x6::zeros(), p: Matrix3x7::zeros(), }
        };

        for line in raw.split('\n') {
            this.process_line(line.trim())?
        }

        Ok(this)
    }

    fn process_line(&mut self, line: &str) -> Result<(), String> {
        if line.is_empty() || line.starts_with('#') {
            return Ok(())
        }

        match line.split_once('=') {
            Some((key, value)) => match key.trim() {
                "kinematics_h" => self.set_h(value.trim()),
                "kinematics_p" => self.set_p(value.trim()),
                _ => Ok(()),
            },
            None => Err(format!("Line '{}' is not in the form `key=value`", line)),
        }
    }

    fn set_h(&mut self, value: &str) -> Result<(), String> {
        let vectors = self.parse_kinematics(value)?;

        match vectors.len() {
            6 => {
                self.kin.h = Matrix3x6::from_columns(&vectors);
                Ok(())
            }

            l => Err(format!("Invalid matrix size for h, {l} instead of 6"))
        }
    }

    fn set_p(&mut self, value: &str) -> Result<(), String> {
        let vectors = self.parse_kinematics(value)?;

        match vectors.len() {
            7 => {
                self.kin.p = Matrix3x7::from_columns(&vectors);
                Ok(())
            }

            l => Err(format!("Invalid matrix size for h, {l} instead of 7"))
        }
    }

    fn parse_kinematics(&mut self, raw: &str) -> Result<Vec<Vector3<f64>>, String> {
        raw.split(',').map(|element| element.trim().split('+').map(|term| self.parse_term(term.trim())).sum()).collect()
    }

    fn parse_term(&mut self, term: &str) -> Result<Vector3<f64>, String> {
        let mut scalar = 1.0;
        let mut vector = None;

        for component in term.split('*') {
            match component.trim() {
                "i" => match vector {
                    None => vector = Some(Vector3::x()),
                    Some(_) => return Err(format!("Failed to express kinematic parameters as linear combinations of unit vectors at {term}"))
                },

                "j" => match vector {
                    None => vector = Some(Vector3::y()),
                    Some(_) => return Err(format!("Failed to express kinematic parameters as linear combinations of unit vectors at {term}"))
                },

                "k" => match vector {
                    None => vector = Some(Vector3::z()),
                    Some(_) => return Err(format!("Failed to express kinematic parameters as linear combinations of unit vectors at {term}"))
                },

                s => scalar *= s.parse::<f64>().or(Err(format!("Invalid float {s}")))?,
            }
        }

        Ok(match vector {
            Some(v) => v * scalar,
            _ => Vector3::zeros(),
        })
    }
}

impl IkFast {
    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
        }
    }

    fn get_kin() -> Kinematics<6, 7> {
        let config = IkFastConfig::load(CONFIG_PATH).expect("Failed to parse kinematic parameters");
        config.kin
    }
}

impl SetupIk for IkFast {
    fn setup(&mut self) {
        let q = Vector6::zeros().map(|_: f64| random_angle());
        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        hardcoded_setup_from_string(raw, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        self.q = compute_ik(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            calculate_ik_error(&self.kin, &self.r, &self.t, q)
        }).reduce(f64::min).unwrap_or(NAN)
    }

    fn ls_count(&self) -> usize {
        unimplemented!()
    }

    fn solution_count(&self) -> usize {
        self.q.len()
    }

    fn name(&self) -> &'static str {
        concat!("IKFAST ", include!("../lib_ikfast/name.rs"))
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

extern "C" {
    fn compute_ik_proxy(rotation: *const f64, translation: *const f64, q: *mut f64) -> usize;
}

pub fn compute_ik(rotation: &Matrix3<f64>, translation: &Vector3<f64>) -> Vec<Vector6<f64>> {
    let rotation_transposed = rotation.transpose();
    let rotation = rotation_transposed.as_slice();
    let translation = translation.as_slice();

    let mut q_data = [0.0; 6 * 8];
    let mut q: Vec<Vector6<f64>> = Vec::new();

    let solutions = unsafe {
        compute_ik_proxy(rotation.as_ptr(), translation.as_ptr(), q_data.as_mut_ptr())
    };

    for i in 0..solutions {
        q.push(Vector6::from_column_slice(&q_data[i * 6 .. (i + 1) * 6]));
    }

    q
}
