use {
    std::{
        io::{ self, Lines, BufReader, BufRead },
        fs::File,
        time::Instant,
    },

    crate::subproblems::setups::{
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
};

#[test]
fn time_separate() {
    let setups: Vec<Box<dyn SetupDynamic>> = vec![
        Box::new(Subproblem1Setup::new()),
        Box::new(Subproblem2Setup::new()),
        Box::new(Subproblem2ExtendedSetup::new()),
        Box::new(Subproblem3Setup::new()),
        Box::new(Subproblem4Setup::new()),
        Box::new(Subproblem5Setup::new()),
        Box::new(Subproblem6Setup::new()),
    ];

    for mut setup in setups {
        let name = setup.name();
        let input_path = "data/".to_owned() + &name.replace(' ', "_") + ".csv";

        let mut total_time = 0;
        let mut n = 0;

        for line in read_lines(&input_path).unwrap().skip(1) {
            setup.setup_from_str(&line.unwrap());

            let start = Instant::now();

            setup.run();
            total_time += start.elapsed().as_nanos();
            n += 1;
        }

        println!("Separate\t{}\t{} ns", name, total_time / n);
    }
}

#[test]
fn time_batch() {
    time_subproblem_batched::<Subproblem1Setup>();
    time_subproblem_batched::<Subproblem2Setup>();
    time_subproblem_batched::<Subproblem2ExtendedSetup>();
    time_subproblem_batched::<Subproblem3Setup>();
    time_subproblem_batched::<Subproblem4Setup>();
    time_subproblem_batched::<Subproblem5Setup>();
    time_subproblem_batched::<Subproblem6Setup>();
}

fn time_subproblem_batched<S: SetupStatic + SetupDynamic>() {
    let name = <S as SetupStatic>::name();
    let input_path = "data/".to_owned() + &name.replace(' ', "_") + ".csv";

    let mut setups = Vec::new();

    for line in read_lines(&input_path).unwrap().skip(1) {
        setups.push(S::new());
        setups.last_mut().unwrap().setup_from_str(&line.unwrap());
    }

    let length = setups.len() as u128;
    let start = Instant::now();

    for mut setup in setups {
        setup.run();
    }

    println!("Batched \t{}\t{} ns", name, start.elapsed().as_nanos() / length);
}

fn read_lines(path: &str) -> io::Result<Lines<BufReader<File>>> {
    let file = File::open(path)?;
    Ok(BufReader::new(file).lines())
}
