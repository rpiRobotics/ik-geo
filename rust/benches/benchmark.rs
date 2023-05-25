/*
use {
    linear_subproblem_solutions_rust::{
        inverse_kinematics::{
            hardcoded::setups::{ Irb6640, KukaR800FixedQ3, RrcFixedQ6, YumiFixedQ3, Ur5, ThreeParallelBot, TwoParallelBot, SphericalBot },

            setups::{
                SetupIk,
                SphericalTwoParallelSetup,
                SphericalTwoIntersectingSetup,
                SphericalSetup,
                ThreeParallelTwoIntersectingSetup,
                ThreeParallelSetup,
                TwoParallelSetup,
                TwoIntersectingSetup,
                GenSixDofSetup,
            },
        },

        subproblems::setups::{
            SetupDynamic,
            SetupStatic,
            Subproblem1Setup,
            Subproblem2Setup,
            Subproblem2ExtendedSetup,
            Subproblem3Setup,
            Subproblem4Setup,
            Subproblem5Setup,
            Subproblem6Setup,
        },
    },

    criterion::{
        Criterion,
        criterion_group,
        criterion_main,
    },
};

pub fn subproblem1benchmark(c: &mut Criterion) {
    let mut setup = Subproblem1Setup::new();
    setup.setup();
    c.bench_function("Subproblem 1 Benchmark", |b| b.iter(|| setup.run()));
}

pub fn subproblem2benchmark(c: &mut Criterion) {
    let mut setup = Subproblem2Setup::new();
    setup.setup();
    c.bench_function("Subproblem 2 Benchmark", |b| b.iter(|| setup.run()));
}

pub fn subproblem2extended_benchmark(c: &mut Criterion) {
    let mut setup = Subproblem2ExtendedSetup::new();
    setup.setup();
    c.bench_function("Subproblem 2 Ex Benchmark", |b| b.iter(|| setup.run()));
}

pub fn subproblem3benchmark(c: &mut Criterion) {
    let mut setup = Subproblem3Setup::new();
    setup.setup();
    c.bench_function("Subproblem 3 Benchmark", |b| b.iter(|| setup.run()));
}

pub fn subproblem4benchmark(c: &mut Criterion) {
    let mut setup = Subproblem4Setup::new();
    setup.setup();
    c.bench_function("Subproblem 4 Benchmark", |b| b.iter(|| setup.run()));
}

pub fn subproblem5benchmark(c: &mut Criterion) {
    let mut setup = Subproblem5Setup::new();
    setup.setup();
    c.bench_function("Subproblem 5 Benchmark", |b| b.iter(|| setup.run()));
}

pub fn subproblem6benchmark(c: &mut Criterion) {
    let mut setup = Subproblem6Setup::new();
    setup.setup();
    c.bench_function("Subproblem 6 Benchmark", |b| b.iter(|| setup.run()));
}

pub fn spherical_two_parallel_benchmark(c: &mut Criterion) {
    let mut setup = SphericalTwoParallelSetup::new();
    setup.setup();
    c.bench_function("Ik Spherical 2 Parallel", |b| b.iter(|| setup.run()));
}

pub fn spherical_two_intersecting_benchmark(c: &mut Criterion) {
    let mut setup = SphericalTwoIntersectingSetup::new();
    setup.setup();
    c.bench_function("Ik Spherical 2 Intersecting", |b| b.iter(|| setup.run()));
}

pub fn spherical_benchmark(c: &mut Criterion) {
    let mut setup = SphericalSetup::new();
    setup.setup();
    c.bench_function("Ik Spherical", |b| b.iter(|| setup.run()));
}

pub fn three_parallel_two_intersecting_benchmark(c: &mut Criterion) {
    let mut setup = ThreeParallelTwoIntersectingSetup::new();
    setup.setup();
    c.bench_function("Ik 3 Parallel 2 Intersecting", |b| b.iter(|| setup.run()));
}

pub fn three_parallel_benchmark(c: &mut Criterion) {
    let mut setup = ThreeParallelSetup::new();
    setup.setup();
    c.bench_function("Ik 3 Parallel", |b| b.iter(|| setup.run()));
}

pub fn two_parallel_benchmark(c: &mut Criterion) {
    let mut setup = TwoParallelSetup::new();
    setup.setup();
    c.bench_function("Ik 2 Parallel", |b| b.iter(|| setup.run()));
}

pub fn two_intersecting_benchmark(c: &mut Criterion) {
    let mut setup = TwoIntersectingSetup::new();
    setup.setup();
    c.bench_function("Ik 2 Intersecting", |b| b.iter(|| setup.run()));
}

pub fn gen_six_dof_benchmark(c: &mut Criterion) {
    let mut setup = GenSixDofSetup::new();
    setup.setup();
    c.bench_function("Gen 6 DOF", |b| b.iter(|| setup.run()));
}

pub fn irb6640_benchmark(c: &mut Criterion) {
    let mut setup = Irb6640::new();
    setup.setup();
    c.bench_function("IRB 6640 (hardcoded)", |b| b.iter(|| setup.run()));
}

pub fn kuka_r800_fixed_q3_benchmark(c: &mut Criterion) {
    let mut setup = KukaR800FixedQ3::new();
    setup.setup();
    c.bench_function("KUKA R800 Fixed Q3 (hardcoded)", |b| b.iter(|| setup.run()));
}

pub fn rrc_fixed_q6_benchmark(c: &mut Criterion) {
    let mut setup = RrcFixedQ6::new();
    setup.setup();
    c.bench_function("RRC Fixed Q6 (hardcoded)", |b| b.iter(|| setup.run()));
}

pub fn yumi_fixed_q3_benchmark(c: &mut Criterion) {
    let mut setup = YumiFixedQ3::new();
    setup.setup();
    c.bench_function("Yumi Fixed Q3 (hardcoded)", |b| b.iter(|| setup.run()));
}

pub fn ur5_benchmark(c: &mut Criterion) {
    let mut setup = Ur5::new();
    setup.setup();
    c.bench_function("UR5 (hardcoded)", |b| b.iter(|| setup.run()));
}

pub fn three_parallel_bot_benchmark(c: &mut Criterion) {
    let mut setup = ThreeParallelBot::new();
    setup.setup();
    c.bench_function("3 Parallel Bot (hardcoded)", |b| b.iter(|| setup.run()));
}

pub fn two_parallel_bot_benchmark(c: &mut Criterion) {
    let mut setup = TwoParallelBot::new();
    setup.setup();
    c.bench_function("2 Parallel Bot (hardcoded)", |b| b.iter(|| setup.run()));
}

pub fn spherical_bot_benchmark(c: &mut Criterion) {
    let mut setup = SphericalBot::new();
    setup.setup();
    c.bench_function("Spherical Bot (hardcoded)", |b| b.iter(|| setup.run()));
}

criterion_group!(
    benches,
    subproblem1benchmark,
    subproblem2benchmark,
    subproblem2extended_benchmark,
    subproblem3benchmark,
    subproblem4benchmark,
    subproblem5benchmark,
    subproblem6benchmark,
    spherical_two_parallel_benchmark,
    spherical_two_intersecting_benchmark,
    three_parallel_two_intersecting_benchmark,
    three_parallel_benchmark,
    two_parallel_benchmark,
    two_intersecting_benchmark,
    gen_six_dof_benchmark,
    irb6640_benchmark,
    kuka_r800_fixed_q3_benchmark,
    rrc_fixed_q6_benchmark,
    yumi_fixed_q3_benchmark,
    ur5_benchmark,
    three_parallel_bot_benchmark,
    two_parallel_bot_benchmark,
    spherical_bot_benchmark,
);

criterion_main!(benches);
 */

fn main() {

}
