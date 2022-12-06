use crate::{
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

    inverse_kinematics::setups::SphericalTwoParallelSetup,
};

const TEST_ITERATIONS: usize = 1000;

#[test]
fn run_tests() {
    // const ERROR_THRESHOLD: f64 = 1e-5;

    let setups: Vec<Box<dyn SetupDynamic>> = vec![
        Box::new(Subproblem6Setup::new()),
        Box::new(Subproblem5Setup::new()),
        Box::new(Subproblem4Setup::new()),
        Box::new(Subproblem3Setup::new()),
        Box::new(Subproblem2ExtendedSetup::new()),
        Box::new(Subproblem2Setup::new()),
        Box::new(Subproblem1Setup::new()),

        Box::new(SphericalTwoParallelSetup::new()),
    ];

    for mut setup in setups {
        let mut total_error = 0.0;

        for _ in 0..TEST_ITERATIONS {
            setup.setup();
            setup.run();

            let error = setup.error();
            // assert!(error <= ERROR_THRESHOLD, "{} error was at: {}", setup.name(), error);
            total_error += error;
        }

        println!("{}\tAvg Error:\t{}", setup.name(), total_error / TEST_ITERATIONS as f64);
    }
}

#[test]
fn run_tests_ls() {
    let setups: Vec<Box<dyn SetupDynamic>> = vec![
        Box::new(Subproblem1Setup::new()),
        Box::new(Subproblem2Setup::new()),
        Box::new(Subproblem3Setup::new()),
        Box::new(Subproblem4Setup::new()),
    ];

    for mut setup in setups {
        for _ in 0..TEST_ITERATIONS {
            setup.setup_ls();
            setup.run();

            assert!(setup.is_at_local_min(), "{} solution was not at local minimum", setup.name());
        }
    }
}
