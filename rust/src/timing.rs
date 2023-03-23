use crate::inverse_kinematics::setups::TwoParallelSetup;

use {
    std::{
        io::{ self, Lines, BufReader, BufRead, Write },
        fs::File,
        time::Instant,
    },

    crate::{
        subproblems::setups::{
            SetupStatic,
            SetupDynamic,
            Subproblem1Setup,
            Subproblem2Setup,
            Subproblem2ExtendedSetup,
            Subproblem3Setup,
            Subproblem4Setup,
            Subproblem5Setup,
            Subproblem6Setup,
        },

        inverse_kinematics::setups::{
            SetupIk,
            SphericalSetup,
            SphericalTwoIntersectingSetup,
            SphericalTwoParallelSetup,
            ThreeParallelSetup,
            ThreeParallelTwoIntersectingSetup,
        },
    },
};

#[test]
fn time_separate() {
    println!();

    time_subproblem_separate::<Subproblem1Setup>();
    time_subproblem_separate::<Subproblem2Setup>();
    time_subproblem_separate::<Subproblem2ExtendedSetup>();
    time_subproblem_separate::<Subproblem3Setup>();
    time_subproblem_separate::<Subproblem4Setup>();
    time_subproblem_separate::<Subproblem5Setup>();
    time_subproblem_separate::<Subproblem6Setup>();
}

#[test]
fn time_batch() {
    println!();

    time_subproblem_batched::<Subproblem1Setup>();
    time_subproblem_batched::<Subproblem2Setup>();
    time_subproblem_batched::<Subproblem2ExtendedSetup>();
    time_subproblem_batched::<Subproblem3Setup>();
    time_subproblem_batched::<Subproblem4Setup>();
    time_subproblem_batched::<Subproblem5Setup>();
    time_subproblem_batched::<Subproblem6Setup>();
}

#[test]
fn time_separate_ik() {
    println!();

    time_ik_separate::<SphericalSetup>();
    time_ik_separate::<SphericalTwoIntersectingSetup>();
    time_ik_separate::<SphericalTwoParallelSetup>();
    time_ik_separate::<ThreeParallelSetup>();
    time_ik_separate::<ThreeParallelTwoIntersectingSetup>();
    time_ik_separate::<TwoParallelSetup>();
}

#[test]
fn time_batch_ik() {
    println!();

    time_ik_batched::<SphericalSetup>();
    time_ik_batched::<SphericalTwoIntersectingSetup>();
    time_ik_batched::<SphericalTwoParallelSetup>();
    time_ik_batched::<ThreeParallelSetup>();
    time_ik_batched::<ThreeParallelTwoIntersectingSetup>();
    time_ik_batched::<TwoParallelSetup>();
}

fn time_subproblem_batched<S: SetupStatic + SetupDynamic>() {
    let name = <S as SetupStatic>::name();
    let input_path = "data/".to_owned() + &name.replace(' ', "_") + ".csv";
    let output_path = "data/out/".to_owned() + &name.replace(' ', "_") + "_batched.csv";

    let mut file = File::create(output_path).unwrap();
    let mut setups = Vec::new();
    let mut results = Vec::new();

    for line in read_lines(&input_path).unwrap().skip(1) {
        setups.push(S::new());
        setups.last_mut().unwrap().setup_from_str(&line.unwrap());
    }

    let start = Instant::now();

    for setup in setups.iter_mut() {
        setup.run();
    }

    let time_per_iteration = start.elapsed().as_nanos() / setups.len() as u128;

    for setup in setups.iter() {
        results.push(setup.write_output())
    }

    println!("Batched \t{}\t{} ns", name, time_per_iteration);

    file.write(&results.join("\n").as_bytes()).unwrap();
}

fn time_subproblem_separate<S: SetupStatic + SetupDynamic>() {
    let name = <S as SetupStatic>::name();
    let input_path = "data/".to_owned() + &name.replace(' ', "_") + ".csv";
    let output_path = "data/out/".to_owned() + &name.replace(' ', "_") + "_separate.csv";

    let mut file = File::create(output_path).unwrap();
    let mut setups = Vec::new();
    let mut results = Vec::new();

    for line in read_lines(&input_path).unwrap().skip(1) {
        setups.push(S::new());
        setups.last_mut().unwrap().setup_from_str(&line.unwrap());
    }

    let mut total_time = 0;

    for setup in setups.iter_mut() {
        let start = Instant::now();
        setup.run();
        total_time += start.elapsed().as_nanos();
    }

    for setup in setups.iter() {
        results.push(setup.write_output())
    }

    println!("Separate\t{}\t{} ns", name, total_time / setups.len() as u128);

    file.write(&results.join("\n").as_bytes()).unwrap();
}

fn time_ik_batched<S: SetupStatic + SetupIk>() {
    let name = <S as SetupStatic>::name();
    let input_path = "data/Ik".to_owned() + &name.replace(' ', "") + ".csv";
    let output_path = "data/out/Ik".to_owned() + &name.replace(' ', "") + "_batched.csv";

    let mut file = File::create(output_path).unwrap();
    let mut setups = Vec::new();
    let mut results = Vec::new();

    for line in read_lines(&input_path).unwrap().skip(1) {
        setups.push(S::new());
        setups.last_mut().unwrap().setup_from_str(&line.unwrap());
    }

    let start = Instant::now();

    for setup in setups.iter_mut() {
        setup.run();
    }

    let time_per_iteration = start.elapsed().as_nanos() / setups.len() as u128;

    for setup in setups.iter() {
        results.push(setup.write_output())
    }

    println!("Batched \t{}\t{} ns", name, time_per_iteration);

    file.write(&results.join("\n").as_bytes()).unwrap();
}

fn time_ik_separate<S: SetupStatic + SetupIk>() {
    let name = <S as SetupStatic>::name();
    let input_path = "data/Ik".to_owned() + &name.replace(' ', "") + ".csv";
    let output_path = "data/out/Ik".to_owned() + &name.replace(' ', "") + "_separate.csv";

    let mut file = File::create(output_path).unwrap();
    let mut setups = Vec::new();
    let mut results = Vec::new();

    for line in read_lines(&input_path).unwrap().skip(1) {
        setups.push(S::new());
        setups.last_mut().unwrap().setup_from_str(&line.unwrap());
    }

    let mut total_time = 0;

    for setup in setups.iter_mut() {
        let start = Instant::now();
        setup.run();
        total_time += start.elapsed().as_nanos();
    }

    for setup in setups.iter() {
        results.push(setup.write_output())
    }

    println!("Separate\t{}\t{} ns", name, total_time / setups.len() as u128);

    file.write(&results.join("\n").as_bytes()).unwrap();
}

fn read_lines(path: &str) -> io::Result<Lines<BufReader<File>>> {
    let file = File::open(path)?;
    Ok(BufReader::new(file).lines())
}
