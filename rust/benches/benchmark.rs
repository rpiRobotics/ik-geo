use {
    linear_subproblem_solutions_rust::subproblems::{
        subproblem1,
        subproblem2,
        subproblem2extended,
        subproblem3,
        subproblem4,
    },

    nalgebra::Vector3,

    criterion::{
        Criterion,
        black_box,
        criterion_group,
        criterion_main,
    },
};

fn random_vector3() -> Vector3<f64> {
    Vector3::new(random(), random(), random())
}

fn random_norm_vector3() -> Vector3<f64> {
    Vector3::new(random(), random(), random()).normalize()
}

fn random() -> f64 {
    fastrand::f64() * 2.0 - 1.0
}

pub fn subproblem1benchmark(c: &mut Criterion) {
    let p1 = random_vector3();
    let p2 = random_vector3();
    let k = random_norm_vector3();

    c.bench_function("Subproblem 1 Benchmark", |b| b.iter(|| subproblem1(
        black_box(&p1),
        black_box(&p2),
        black_box(&k),
    )));
}

pub fn subproblem2benchmark(c: &mut Criterion) {
    let p1 = random_vector3();
    let p2 = random_vector3();
    let k1 = random_norm_vector3();
    let k2 = random_norm_vector3();

    c.bench_function("Subproblem 2 Benchmark", |b| b.iter(|| subproblem2(
        black_box(&p1),
        black_box(&p2),
        black_box(&k1),
        black_box(&k2),
    )));
}

pub fn subproblem2extended_benchmark(c: &mut Criterion) {
    let p0 = random_vector3();
    let p1 = random_vector3();
    let p2 = random_vector3();
    let k1 = random_norm_vector3();
    let k2 = random_norm_vector3();

    c.bench_function("Subproblem 2 Ex Benchmark", |b| b.iter(|| subproblem2extended(
        black_box(&p0),
        black_box(&p1),
        black_box(&p2),
        black_box(&k1),
        black_box(&k2),
    )));
}

pub fn subproblem3benchmark(c: &mut Criterion) {
    let p1 = random_vector3();
    let p2 = random_vector3();
    let k = random_norm_vector3();
    let d = random();

    c.bench_function("Subproblem 3 Benchmark", |b| b.iter(|| subproblem3(
        black_box(&p1),
        black_box(&p2),
        black_box(&k),
        black_box(d),
    )));
}

pub fn subproblem4benchmark(c: &mut Criterion) {
    let h = random_norm_vector3();
    let p = random_vector3();
    let k = random_norm_vector3();
    let d = random();

    c.bench_function("Subproblem 4 Benchmark", |b| b.iter(|| subproblem4(
        black_box(&h),
        black_box(&p),
        black_box(&k),
        black_box(d),
    )));
}

criterion_group!(
    benches,
    subproblem1benchmark,
    subproblem2benchmark,
    subproblem2extended_benchmark,
    subproblem3benchmark,
    subproblem4benchmark,
);

criterion_main!(benches);
