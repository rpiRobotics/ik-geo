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

    inverse_kinematics::{
        setups::{
            SphericalTwoParallelSetup,
            SphericalTwoIntersectingSetup,
            SphericalSetup,
            ThreeParallelTwoIntersectingSetup,
            SetupIk,
            ThreeParallelSetup,
            TwoParallelSetup,
            TwoIntersectingSetup,
            GenSixDofSetup,
        },

        hardcoded::setups::{
            Irb6640,
            KukaR800FixedQ3,
            RrcFixedQ6,
            Ur5,
            ThreeParallelBot,
            TwoParallelBot,
            SphericalBot,
            YumiFixedQ3
        },
    },
};

#[cfg(link_ikfast)]
use crate::ikfast::IkFast;

const TEST_ITERATIONS: usize = 1000;

#[cfg(link_ikfast)]
#[test]
fn ikfast_tests() {
    let mut setup = IkFast::new();

    let mut total_error = 0.0;
    let mut num_iterations = 0;
    let mut nan_count = 0;

    for _ in 0..TEST_ITERATIONS {
        setup.setup();
        setup.run();

        let error = setup.error();

        if error.is_nan() {
            nan_count += 1;
        }
        else {
            // assert!(error <= ERROR_THRESHOLD, "{} error was at: {:.2e}", setup.name(), error);
            total_error += error;
            num_iterations += 1;
        }
    }

    let avg_err = total_error / (num_iterations) as f64;
    let nan_percent = nan_count as f64 * 100.0 / TEST_ITERATIONS as f64;

    println!("{}", setup.name());
    println!("\tAvg min Error:\t{avg_err:.2e}");
    println!("\t% NaN:\t{nan_percent:.2}");
}

#[test]
fn run_tests() {
    let setups: Vec<Box<dyn SetupDynamic>> = vec![
        Box::new(Subproblem6Setup::new()),
        Box::new(Subproblem5Setup::new()),
        Box::new(Subproblem4Setup::new()),
        Box::new(Subproblem3Setup::new()),
        Box::new(Subproblem2ExtendedSetup::new()),
        Box::new(Subproblem2Setup::new()),
        Box::new(Subproblem1Setup::new()),
    ];

    let ik_setups: Vec<Box<dyn SetupIk>> = vec![
        Box::new(SphericalTwoParallelSetup::new()),
        Box::new(SphericalTwoIntersectingSetup::new()),
        Box::new(SphericalSetup::new()),
        Box::new(ThreeParallelTwoIntersectingSetup::new()),
        Box::new(ThreeParallelSetup::new()),
        Box::new(TwoParallelSetup::new()),
        Box::new(TwoIntersectingSetup::new()),
        Box::new(GenSixDofSetup::new()),

        Box::new(Irb6640::new()),
        Box::new(KukaR800FixedQ3::new()),
        Box::new(RrcFixedQ6::new()),
        Box::new(Ur5::new()),
        Box::new(ThreeParallelBot::new()),
        Box::new(TwoParallelBot::new()),
        Box::new(SphericalBot::new()),
        Box::new(YumiFixedQ3::new()),
    ];

    for mut setup in setups {
        let mut total_error = 0.0;
        let mut nan_count = 0;
        let mut ls_count = 0;
        let mut num_iterations = 0;

        for _ in 0..TEST_ITERATIONS {
            setup.setup();
            let is_ls = setup.run_report_info();

            if is_ls {
                ls_count += 1;
            }

            let error = setup.error();

            if error.is_nan() {
                nan_count += 1;
            }
            else {
                // assert!(error <= ERROR_THRESHOLD, "{} error was at: {:.2e}", setup.name(), error);
                total_error += error;
                num_iterations += 1;
            }
        }

        let avg_err = total_error / num_iterations as f64;
        let nan_percent = nan_count as f64 * 100.0 / TEST_ITERATIONS as f64;
        let ls_percent = ls_count as f64 * 100.0 / TEST_ITERATIONS as f64;

        println!("{}", setup.name());
        println!("\tAvg Error:\t{avg_err:.2e}");
        println!("\t% NaN:\t{nan_percent:.2}");
        println!("\t% LS:\t{ls_percent:.2}");
    }

    for mut setup in ik_setups {
        let mut total_error = 0.0;
        let mut num_q_ls = 0;
        let mut num_all_ls = 0;
        let mut total_q_count = 0;
        let mut nan_count = 0;
        let mut num_iterations = 0;

        for _ in 0..TEST_ITERATIONS {
            setup.setup();
            setup.run();

            let error = setup.error();
            let n_ls = setup.ls_count();
            let n_sol = setup.solution_count();

            if error.is_nan() {
                nan_count += 1;
            }
            else {
                // assert!(error <= ERROR_THRESHOLD, "{} error was at: {:.2e}", setup.name(), error);
                total_error += error;
                num_iterations += 1;

                num_q_ls += n_ls;
                total_q_count += n_sol;
                if n_ls == n_sol { num_all_ls += 1 }
            }
        }

        let avg_err = total_error / (num_iterations - num_all_ls) as f64;
        let nan_percent = nan_count as f64 * 100.0 / TEST_ITERATIONS as f64;
        let ls_percent = num_q_ls as f64 * 100.0 / total_q_count as f64;
        let all_ls_percent = num_all_ls as f64 * 100.0 / TEST_ITERATIONS as f64;

        println!("{}", setup.name());
        println!("\tAvg min Error:\t{avg_err:.2e}");
        println!("\t% NaN:\t{nan_percent:.2}");
        println!("\t% LS:\t{ls_percent:.2}");
        println!("\t% All LS:\t{all_ls_percent:.2}");
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
