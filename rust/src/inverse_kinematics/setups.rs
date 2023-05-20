use {
    nalgebra::{ Vector3, Vector6, Matrix3, Matrix3x6 },

    crate::subproblems::{
        setups::SetupStatic,

        auxiliary::{
            random_vector3,
            random_norm_vector3,
            random_angle,
        },
    },

    std::f64::NAN,

    super::{
        auxiliary::{
            Kinematics,
            Matrix3x7,
        },

        spherical_two_parallel,
        spherical_two_intersecting,
        spherical,
        three_parallel_two_intersecting,
        three_parallel,
        two_parallel,
        two_intersecting,
        gen_six_dof,
    },
};

pub trait SetupIk {
    fn setup(&mut self);
    fn setup_from_str(&mut self, raw: &str);
    fn write_output(&self) -> String;
    fn run(&mut self);
    fn error(&self) -> f64;
    fn ls_count(&self) -> usize;
    fn solution_count(&self) -> usize;
    fn name(&self) -> &'static str;
    fn debug(&self, i: usize);
}

pub struct SphericalTwoParallelSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct SphericalTwoIntersectingSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct SphericalSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct ThreeParallelTwoIntersectingSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct ThreeParallelSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct TwoParallelSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct TwoIntersectingSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub struct GenSixDofSetup {
    kin: Kinematics<6, 7>,
    r: Matrix3<f64>,
    t: Vector3<f64>,

    q: Vec<Vector6<f64>>,
    is_ls: Vec<bool>,
}

pub fn calculate_ik_error(kin: &Kinematics<6, 7>, r: &Matrix3<f64>, t: &Vector3<f64>, q: &Vector6<f64>) -> f64 {
    let (r_t, t_t) = kin.forward_kinematics(q);
    (r_t - r).norm() + (t_t - t).norm()
}

fn ik_setup_from_string(raw: &str, kin: &mut Kinematics<6, 7>, r: &mut Matrix3<f64>, t: &mut Vector3<f64>) {
    let data: Vec<f64> = raw.split(',').map(|s| s.parse().unwrap()).collect();

    kin.h = Matrix3x6::from_columns(&[
        Vector3::new(data[0], data[1], data[2]),
        Vector3::new(data[3], data[4], data[5]),
        Vector3::new(data[6], data[7], data[8]),
        Vector3::new(data[9], data[10], data[11]),
        Vector3::new(data[12], data[13], data[14]),
        Vector3::new(data[15], data[16], data[17]),
    ]);

    kin.p = Matrix3x7::from_columns(&[
        Vector3::new(data[18], data[19], data[20]),
        Vector3::new(data[21], data[22], data[23]),
        Vector3::new(data[24], data[25], data[26]),
        Vector3::new(data[27], data[28], data[29]),
        Vector3::new(data[30], data[31], data[32]),
        Vector3::new(data[33], data[34], data[35]),
        Vector3::new(data[36], data[37], data[38]),
    ]);

    *r = Matrix3::new(
        data[39], data[40], data[41],
        data[42], data[43], data[44],
        data[45], data[46], data[47],
    );

    *t = Vector3::new(
        data[48],
        data[49],
        data[50]
    );
}

pub fn ik_write_output(q: &Vec<Vector6<f64>>) -> String {
    q.iter()
        .map(|q| q.iter().map(|x| x.to_string())
        .collect::<Vec<String>>().join(",")).collect::<Vec<String>>()
        .join(",")
}

impl SetupStatic for SphericalTwoParallelSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Spherical two Parallel"
    }
}

impl SetupStatic for SphericalTwoIntersectingSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Spherical Two Intersecting"
    }
}

impl SetupStatic for SphericalSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Spherical"
    }
}

impl SetupStatic for ThreeParallelTwoIntersectingSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Three Parallel Two Intersecting"
    }
}

impl SetupStatic for ThreeParallelSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Three Parallel"
    }
}

impl SetupStatic for TwoParallelSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Two Parallel"
    }
}

impl SetupStatic for TwoIntersectingSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Two Intersecting"
    }
}

impl SetupStatic for GenSixDofSetup {
    fn new() -> Self {
        Self {
            kin: Kinematics::new(),
            r: Matrix3::zeros(),
            t: Vector3::zeros(),

            q: Vec::new(),
            is_ls: Vec::new(),
        }
    }

    fn name() -> &'static str {
        "Gen Six DOF"
    }
}

impl SetupIk for SphericalTwoParallelSetup {
    fn setup(&mut self) {
        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        let h_column_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_column_1);

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical_two_parallel(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}

impl SetupIk for SphericalTwoIntersectingSetup {
    fn setup(&mut self) {
        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            Vector3::zeros(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical_two_intersecting(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}

impl SetupIk for SphericalSetup {
    fn setup(&mut self) {
        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = spherical(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}

impl SetupIk for ThreeParallelTwoIntersectingSetup {
    fn setup(&mut self) {
        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        let h_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_1);
        self.kin.h.set_column(3, &h_1);

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = three_parallel_two_intersecting(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}

impl SetupIk for ThreeParallelSetup {
    fn setup(&mut self) {
        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        let h_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_1);
        self.kin.h.set_column(3, &h_1);

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
        ]);

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = three_parallel(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}

impl SetupIk for TwoParallelSetup {
    fn setup(&mut self) {
        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        let h_1: Vector3<f64> = self.kin.h.column(1).into();
        self.kin.h.set_column(2, &h_1);

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
        ]);

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = two_parallel(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}

impl SetupIk for TwoIntersectingSetup {
    fn setup(&mut self) {
        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        let q = Vector6::zeros().map(|_: f64| random_angle());

        self.kin.p = Matrix3x7::from_columns(&[
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            random_vector3(),
            Vector3::zeros(),
            random_vector3(),
        ]);

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = two_intersecting(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}

impl SetupIk for GenSixDofSetup {
    fn setup(&mut self) {
        let q = Vector6::zeros().map(|_: f64| random_angle());

        for i in 0..self.kin.h.ncols() {
            self.kin.h.set_column(i, &random_norm_vector3());
        }

        for i in 0..self.kin.p.ncols() {
            self.kin.p.set_column(i, &random_vector3());
        }

        (self.r, self.t) = self.kin.forward_kinematics(&q);
    }

    fn setup_from_str(&mut self, raw: &str) {
        ik_setup_from_string(raw, &mut self.kin, &mut self.r, &mut self.t);
    }

    fn write_output(&self) -> String {
        ik_write_output(&self.q)
    }

    fn run(&mut self) {
        (self.q, self.is_ls) = gen_six_dof(&self.r, &self.t, &self.kin);
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
        println!("{i}\nr{}t{}h{}p{}", self.r, self.t, self.kin.h, self.kin.p);
    }
}
