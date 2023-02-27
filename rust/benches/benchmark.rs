use linear_subproblem_solutions_rust::inverse_kinematics::setups::{ThreeParallelTwoIntersectingSetup, ThreeParallelSetup};
#[cfg(link_ikfast)]
use linear_subproblem_solutions_rust::{
    ikfast,
    nalgebra::Vector6,
    inverse_kinematics::{ hardcoded::setups::Irb6640 },
    subproblems::auxiliary::random_angle,
};

use {
    linear_subproblem_solutions_rust::{
        inverse_kinematics::setups::{
            SetupIk,
            SphericalTwoParallelSetup,
            SphericalTwoIntersectingSetup,
            SphericalSetup,
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

#[cfg(link_ikfast)]
pub fn ikfast_kuka_kr30l16_benchmark(c: &mut Criterion) {
    let q = Vector6::zeros().map(|_: f64| random_angle());
    let (r, t) = Irb6640::get_kin().forward_kinematics(&q);

    c.bench_function("IkFast KUKA KR 30 L16", |b| b.iter(|| ikfast::kuka_kr30l16(&r, &t)));
}

#[cfg(link_ikfast)]
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
    spherical_benchmark,
    three_parallel_two_intersecting_benchmark,
    three_parallel_benchmark,
    ikfast_kuka_kr30l16_benchmark,
);

#[cfg(not(link_ikfast))]
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
    spherical_benchmark,
);

criterion_main!(benches);
