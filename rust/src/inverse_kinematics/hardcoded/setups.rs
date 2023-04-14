use {
    std::f64::{ NAN, consts::PI },

    nalgebra::{ Vector3, Vector6, Matrix3, Matrix3x6 },

    super::{ irb6640, kuka_r800_fixed_q3, ur5, three_parallel_bot, two_parallel_bot },

    crate::{
        inverse_kinematics::{
            auxiliary::{
                Kinematics,
                Matrix3x7,
                Matrix3x8,
            },

            setups::{ SetupIk, calculate_ik_error },
        },

        subproblems::{ Vector7, auxiliary::random_angle },
    },
};

pub struct Irb6640 {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct KukaR800FixedQ3 {
    kin: Kinematics<7, 8>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct Ur5 {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct ThreeParallelBot {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct TwoParallelBot {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

impl Irb6640 {
    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    pub fn get_kin() -> Kinematics<6, 7> {
        let mut kin = Kinematics::new();

        let zv = Vector3::zeros();
        let ex = Vector3::x();
        let ey = Vector3::y();
        let ez = Vector3::z();

        kin.h = Matrix3x6::from_columns(&[ez, ey, ey, ex, ey, ex]);
        kin.p = Matrix3x7::from_columns(&[zv, 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ex + 0.2 * ez, zv, zv, 0.2 * ex]);

        kin
    }
}

impl KukaR800FixedQ3 {
    const Q3: f64 = PI / 6.0;

    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),

            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    pub fn get_kin() -> Kinematics<7, 8> {
        let mut kin = Kinematics::new();

        let zv = Vector3::zeros();
        let ey = Vector3::y();
        let ez = Vector3::z();

        kin.h = Matrix3x7::from_columns(&[ez, ey, ez, -ey, ez, ey, ez]);
        kin.p = Matrix3x8::from_columns(&[(0.15 + 0.19) * ez, zv, 0.21 * ez, 0.19 * ez, (0.21 + 0.19) * ez, zv, zv, (0.081 + 0.045) * ez]);

        kin
    }

    pub fn get_kin_partial() -> (Kinematics<6, 7>, Matrix3<f64>) {
        let kin = Self::get_kin();
        kin.forward_kinematics_partial(Self::Q3, 2, &Matrix3::identity())
    }
}

impl Ur5 {
    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    pub fn get_kin() -> Kinematics<6, 7> {
        let mut kin = Kinematics::new();

        let ex = Vector3::x();
        let ey = Vector3::y();
        let ez = Vector3::z();

        kin.h = Matrix3x6::from_columns(&[ez, ey, ey, ey, -ez, ey]);
        kin.p = Matrix3x7::from_columns(&[0.089159*ez, 0.1358*ey, -0.1197*ey + 0.425*ex, 0.3922*ex, 0.093*ey, -0.0946*ez, 0.0823*ey]);

        kin
    }
}

impl ThreeParallelBot {
    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    pub fn get_kin() -> Kinematics<6, 7> {
        let mut kin = Kinematics::new();

        let ex = Vector3::x();
        let ey = Vector3::y();
        let ez = Vector3::z();

        kin.h = Matrix3x6::from_columns(&[ez, ex, ex, ex, ez, ex]);
        kin.p = Matrix3x7::from_columns(&[ez, ey, ey, ey, ey, ey + ex, ex]);

        kin
    }
}

impl TwoParallelBot {
    pub fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    pub fn get_kin() -> Kinematics<6, 7> {
        let mut kin = Kinematics::new();

        let ex = Vector3::x();
        let ey = Vector3::y();
        let ez = Vector3::z();

        let es = (ex + ez).normalize();

        kin.h = Matrix3x6::from_columns(&[ez, ex, ex, ez, ex, es]);
        kin.p = Matrix3x7::from_columns(&[ez, ey, ey, ey, ey, ey, ez]);

        kin
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
        (self.q, self.is_ls) = irb6640(&self.r, &self.t)
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            calculate_ik_error(&self.kin, &self.r, &self.t, q)
        }).reduce(f64::min).unwrap_or(NAN)
    }

    fn ls_count(&self) -> usize {
        self.is_ls.iter().filter(|b| **b).count()
    }

    fn solution_count(&self) -> usize {
        self.is_ls.len()
    }
    fn name(&self) -> &'static str {
        "IRB 6640"
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for KukaR800FixedQ3 {
    fn setup(&mut self) {
        let mut q = Vector7::zeros().map(|_: f64| random_angle());
        q[2] = Self::Q3;
        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, _raw: &str) {
        unimplemented!()
    }

    fn write_output(&self) -> String {
        unimplemented!()
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = kuka_r800_fixed_q3(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            let q_e = Vector7::from_column_slice(&[
                q[0],
                q[1],
                Self::Q3,
                q[2],
                q[3],
                q[4],
                q[5],
            ]);

            let (r_t, t_t) = self.kin.forward_kinematics(&q_e);
            (r_t - self.r).norm() + (t_t - self.t).norm()
        }).reduce(f64::min).unwrap_or(NAN)
    }

    fn ls_count(&self) -> usize {
        self.is_ls.iter().filter(|b| **b).count()
    }

    fn solution_count(&self) -> usize {
        self.is_ls.len()
    }

    fn name(&self) -> &'static str {
        "KUKA R800 Fixed Q3"
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for Ur5 {
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
        (self.q, self.is_ls) = ur5(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            calculate_ik_error(&self.kin, &self.r, &self.t, q)
        }).reduce(f64::min).unwrap_or(NAN)
    }

    fn ls_count(&self) -> usize {
        self.is_ls.iter().filter(|b| **b).count()
    }

    fn solution_count(&self) -> usize {
        self.is_ls.len()
    }
    fn name(&self) -> &'static str {
        "UR5"
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for ThreeParallelBot {
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
        (self.q, self.is_ls) = three_parallel_bot(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            calculate_ik_error(&self.kin, &self.r, &self.t, q)
        }).reduce(f64::min).unwrap_or(NAN)
    }

    fn ls_count(&self) -> usize {
        self.is_ls.iter().filter(|b| **b).count()
    }

    fn solution_count(&self) -> usize {
        self.is_ls.len()
    }
    fn name(&self) -> &'static str {
        "Three Parallel Bot"
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for TwoParallelBot {
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
        (self.q, self.is_ls) = two_parallel_bot(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            calculate_ik_error(&self.kin, &self.r, &self.t, q)
        }).reduce(f64::min).unwrap_or(NAN)
    }

    fn ls_count(&self) -> usize {
        self.is_ls.iter().filter(|b| **b).count()
    }

    fn solution_count(&self) -> usize {
        self.is_ls.len()
    }
    fn name(&self) -> &'static str {
        "Two Parallel Bot"
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}
