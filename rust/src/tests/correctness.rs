use super::setups::{ Setup, Subproblem1Setup, Subproblem2Setup };

const TEST_ITERATIONS: usize = 1000;

#[test]
fn run_tests() {
    const ERROR_THRESHOLD: f64 = 1e-12;

    let setups: Vec<Box<dyn Setup>> = vec![
        Box::new(Subproblem1Setup::new()),
        Box::new(Subproblem2Setup::new()),
    ];

    for mut setup in setups {
        let mut total_error = 0.0;

        for _ in 0..TEST_ITERATIONS {
            setup.setup();
            setup.run();

            let error = setup.error();
            assert!(error <= ERROR_THRESHOLD);
            total_error += error;
        }

        println!("{} Avg Error:\t{}", setup.name(), total_error / TEST_ITERATIONS as f64);
    }
}

#[test]
fn run_tests_ls() {
    let setups: Vec<Box<dyn Setup>> = vec![
        Box::new(Subproblem1Setup::new()),
        Box::new(Subproblem2Setup::new()),
    ];

    for mut setup in setups {
        for _ in 0..TEST_ITERATIONS {
            setup.setup_ls();
            setup.run();

            assert!(setup.is_at_local_min());
        }
    }
}
