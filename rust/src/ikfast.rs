use {
    crate::{
        inverse_kinematics::{
            self,
            auxiliary::{ Kinematics, Matrix3x7 },
            setups::SetupIk
        },

        subproblems::auxiliary::random_angle
    },

    nalgebra::{ Matrix3, Vector3, Vector6, Matrix3x6 },
};

pub struct KukaKr30Setup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
}

pub struct Irb6640 {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
}

impl KukaKr30Setup {
    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
        }
    }

    fn get_kin() -> Kinematics<6, 7> {
        let mut kin = Kinematics::new();

        let zv = Vector3::zeros();
        let ex = Vector3::x();
        let ey = Vector3::y();
        let ez = Vector3::z();


        kin.h = Matrix3x6::from_columns(&[-ez, -ey, -ey, -ex, -ey, -ex]);
        kin.p = Matrix3x7::from_columns(&[zv, 0.35 * ex + 0.815 * ez, 1.2 * ez, 0.145 * ez + 1.545 * ex, zv, zv, 0.158 * ex]);

        kin
    }
}

impl Irb6640 {
    pub fn new() -> Self {
        Self {
            kin: inverse_kinematics::hardcoded::setups::Irb6640::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
        }
    }
}

impl SetupIk for KukaKr30Setup {
    fn setup(&mut self) {
        let q = Vector6::zeros().map(|_: f64| random_angle());
        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        self.q = kuka_kr30l16(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            let (r_t, t_t) = self.kin.forward_kinematics(q);
            (r_t - self.r).norm() + (t_t - self.t).norm()
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn ls_count(&self) -> usize {
        unimplemented!()
    }

    fn solution_count(&self) -> usize {
        self.q.len()
    }

    fn name(&self) -> &'static str {
        "KUKA KR 30"
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for Irb6640 {
    fn setup(&mut self) {
        let q = Vector6::zeros().map(|_: f64| random_angle());
        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        self.q = kuka_kr30l16(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            let (r_t, t_t) = self.kin.forward_kinematics(q);
            (r_t - self.r).norm() + (t_t - self.t).norm()
        }).sum::<f64>() / (self.q.len() as f64 * 2.0)
    }

    fn ls_count(&self) -> usize {
        unimplemented!()
    }

    fn solution_count(&self) -> usize {
        self.q.len()
    }

    fn name(&self) -> &'static str {
        "IKFast IRB 6640"
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

extern "C" {
    fn kuka_kr30l16_c(rotation: *const f64, translation: *const f64, q: *mut f64) -> usize;
}

pub fn kuka_kr30l16(rotation: &Matrix3<f64>, translation: &Vector3<f64>) -> Vec<Vector6<f64>> {
    let rotation_transposed = rotation.transpose();
    let rotation = rotation_transposed.as_slice();
    let translation = translation.as_slice();

    let mut q_data = [0.0; 6 * 8];
    let mut q: Vec<Vector6<f64>> = Vec::new();

    let solutions = unsafe {
        kuka_kr30l16_c(rotation.as_ptr(), translation.as_ptr(), q_data.as_mut_ptr())
    };

    for i in 0..solutions {
        q.push(Vector6::from_column_slice(&q_data[i * 6 .. (i + 1) * 6]));
    }

    q
}
