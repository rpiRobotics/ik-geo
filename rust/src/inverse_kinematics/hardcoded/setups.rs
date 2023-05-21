use {
    std::f64::{ NAN, consts::PI },

    nalgebra::{ Vector3, Vector6, Matrix3, Matrix3x6 },

    super::{ irb6640, kuka_r800_fixed_q3, ur5, three_parallel_bot, two_parallel_bot, rrc_fixed_q6, spherical_bot, yumi_fixed_q3 },

    crate::{
        inverse_kinematics::{
            auxiliary::{
                Kinematics,
                Matrix3x7,
                Matrix3x8,
            },

            setups::{ SetupIk, calculate_ik_error, ik_write_output },
        },

        subproblems::{ Vector7, auxiliary::random_angle, setups::SetupStatic },
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

pub struct RrcFixedQ6 {
    kin: Kinematics<7, 8>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct YumiFixedQ3 {
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

pub struct SphericalBot {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub fn hardcoded_setup_from_string(raw: &str, r: &mut Matrix3<f64>, t: &mut Vector3<f64>) {
    let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

    *r = Matrix3::new(
        data[0], data[1], data[2],
        data[3], data[4], data[5],
        data[6], data[7], data[8],
    );

    *t = Vector3::new(
        data[9],
        data[10],
        data[11]
    );
}

impl Irb6640 {
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

impl RrcFixedQ6 {
    const Q6: f64 = PI / 6.0;

    pub fn get_kin() -> Kinematics<7, 8> {
        let mut kin = Kinematics::new();

        let zv = Vector3::zeros();
        let ex = Vector3::x();
        let ey = Vector3::y();
        let ez = Vector3::z();

        let p01 = zv;
        let p12 = 20.0 * ex - 4.0 * ey;
        let p23 = 4.0 * ey;
        let p34 = 21.5 * ex + 3.375 * ey;
        let p45 = -3.375 * ey;
        let p56 = 21.5 * ex + 3.325 * ey;
        let p67 = -3.325 * ey;
        let p7t = 7.0 * ex;

        kin.p = 0.0254 * Matrix3x8::from_columns(&[p01, p12, p23, p34, p45, p56, p67, p7t]);
        kin.h = Matrix3x7::from_columns(&[ex, ez, ex, ez, ex, ez, ex]);

        kin
    }

    pub fn get_kin_partial() -> (Kinematics<6, 7>, Matrix3<f64>) {
        let kin = Self::get_kin();
        let (mut kin_partial, r_6t) = kin.forward_kinematics_partial(Self::Q6, 5, &Matrix3::identity());

        let zv = Vector3::zeros();
        let alpha = (kin_partial.h.fixed_columns::<2>(4)).pseudo_inverse(1e-12).unwrap() * kin_partial.p.column(5);
        let delta_p_45 = alpha[0] * kin_partial.h.column(4);
        let delta_p_6t = alpha[1] * kin_partial.h.column(5);

        kin_partial.p.set_column(4, &(kin_partial.p.column(4) + delta_p_45));
        kin_partial.p.set_column(5, &zv);
        kin_partial.p.set_column(6, &(kin_partial.p.column(6) + delta_p_6t));

        (kin_partial, r_6t)
    }
}

impl YumiFixedQ3 {
    const Q3: f64 = PI / 6.0;

    pub fn get_kin() -> Kinematics<7, 8> {
        let mut kin = Kinematics::new();

        kin.p = Matrix3x8::from_row_slice(&[
            0.0536, 0.0642, 0.1578, 0.0880, 0.1270, 0.0354, 0.0385, 0.0040,
            0.0725, 0.0527, 0.0406, 0.0011, -0.0877, -0.0712, -0.0087, -0.0043,
            0.4149, 0.0632, 0.0650, 0.0143, -0.0700, -0.0670, -0.0030, -0.0038,
        ]);

        kin.h = Matrix3x7::from_row_slice(&[
            0.8138, 0.1048, 0.8138, 0.1048, 0.5716, 0.1048, 0.5716,
            0.3420, 0.7088, 0.3420, 0.7088, -0.6170, 0.7088, -0.6170,
            0.4698, -0.6976, 0.4698, -0.6976, -0.5410, -0.6976, -0.5410,
        ]);

        for i in 0..kin.h.ncols() {
            kin.h.set_column(i, &kin.h.column(i).normalize());
        }

        kin
    }

    pub fn get_kin_partial() -> (Kinematics<6, 7>, Matrix3<f64>) {
        let kin = Self::get_kin();
        kin.forward_kinematics_partial(Self::Q3, 2, &Matrix3::identity())
    }
}

impl Ur5 {
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

impl SphericalBot {
    pub fn get_kin() -> Kinematics<6, 7> {
        let mut kin = Kinematics::new();

        let zv = Vector3::zeros();
        let ex = Vector3::x();
        let ey = Vector3::y();
        let ez = Vector3::z();

        kin.h = Matrix3x6::from_columns(&[ey, ez, ey, ex, ey, ex]);
        kin.p = Matrix3x7::from_columns(&[zv, ez + ex, ez + ex, ez + ex, zv, zv, ex]);

        kin
    }
}

impl SetupIk for Irb6640 {
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
        <Self as SetupStatic>::name()
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

    fn setup_from_str(&mut self, raw: &str) {
        hardcoded_setup_from_string(raw, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
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
        <Self as SetupStatic>::name()
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for RrcFixedQ6 {
    fn setup(&mut self) {
        let mut q = Vector7::zeros().map(|_: f64| random_angle());
        q[5] = Self::Q6;
        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        hardcoded_setup_from_string(raw, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = rrc_fixed_q6(&self.r, &self.t);
    }

    fn error(&self) -> f64 {
        self.q.iter().map(|q| {
            let q_e = Vector7::from_column_slice(&[
                q[0],
                q[1],
                q[2],
                q[3],
                q[4],
                Self::Q6,
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
        <Self as SetupStatic>::name()
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for YumiFixedQ3 {
    fn setup(&mut self) {
        let mut q = Vector7::zeros().map(|_: f64| random_angle());
        q[2] = Self::Q3;
        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        hardcoded_setup_from_string(raw, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = yumi_fixed_q3(&self.r, &self.t);
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
        <Self as SetupStatic>::name()
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

    fn setup_from_str(&mut self, raw: &str) {
        hardcoded_setup_from_string(raw, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
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
        <Self as SetupStatic>::name()
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

    fn setup_from_str(&mut self, raw: &str) {
        hardcoded_setup_from_string(raw, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
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
        <Self as SetupStatic>::name()
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

    fn setup_from_str(&mut self, raw: &str) {
        hardcoded_setup_from_string(raw, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
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
        <Self as SetupStatic>::name()
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupIk for SphericalBot {
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
        (self.q, self.is_ls) = spherical_bot(&self.r, &self.t);
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
        <Self as SetupStatic>::name()
    }

    fn debug(&self, i: usize) {
        println!("{i}{}{}", self.r, self.t);
    }
}

impl SetupStatic for Irb6640 {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "IRB 6640"
    }
}

impl SetupStatic for KukaR800FixedQ3 {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),

            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "KUKA R800 Fixed Q3"
    }
}

impl SetupStatic for RrcFixedQ6 {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),

            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "RRC Fixed Q6"
    }
}

impl SetupStatic for Ur5 {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "UR5"
    }
}

impl SetupStatic for ThreeParallelBot {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Three Parallel Bot"
    }
}

impl SetupStatic for TwoParallelBot {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Two Parallel Bot"
    }
}

impl SetupStatic for SphericalBot {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Spherical Bot"
    }
}

impl SetupStatic for YumiFixedQ3 {
    fn new() -> Self {
        Self {
            kin: Self::get_kin(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Yumi Fixed Q3"
    }
}
