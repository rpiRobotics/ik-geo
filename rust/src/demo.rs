use std::{env, collections::HashMap};

use linear_subproblem_solutions_rust::inverse_kinematics::hardcoded::{irb6640, kuka_r800_fixed_q3};
use nalgebra::{Matrix3, Vector3, Vector6};

#[derive(Debug, Clone, Copy)]
enum Robot {
    Irb6640,
    KukaR800FixedQ3,
}

struct DemoArgs {
    robot: Robot,
    r: Matrix3<f64>,
    t: Vector3<f64>,
}

fn print_usage(exe_name: &str, robot_names: Vec<&&str>) {
    println!("*** Linear Subproblem Solutions Rust ***");
    println!("\tUsage:      {exe_name} [robot] r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3");
    println!("\tWith cargo: cargo run -- [robot] r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3");
    println!("\t[robot] is one of:");

    for name in robot_names {
        println!("\t\t{name}");
    }

    println!("\tr is the rotation matrix specified row-wise");
    println!("\tt is the translation vector");
}

fn parse_args(robot_map: &HashMap<&str, Robot>) -> Result<DemoArgs, String> {
    fn extract_float<T: Iterator<Item = String>>(iter: &mut T) -> Result<f64, String> {
        let raw = iter.next().ok_or("Not enough arguments".to_owned())?;
        raw.parse().or(Err(format!("Malformed float {raw:?}")))
    }

    let mut args = env::args().skip(1);

    let robot = args.next().ok_or("No robot specified".to_owned())?;
    let robot = *robot_map.get(&robot[..]).ok_or(format!("Invalid robot {robot:?}"))?;

    let mut r = Vec::new();
    let mut t = Vec::new();

    for _ in 0..9 {
        r.push(extract_float(&mut args)?);
    }

    for _ in 0..3 {
        t.push(extract_float(&mut args)?);
    }

    let r = Matrix3::from_vec(r).transpose();
    let t = Vector3::from_vec(t);

    Ok(DemoArgs { robot, r, t })
}

fn compute_ik(demo_args: DemoArgs) -> (Vec<Vector6<f64>>, Vec<bool>) {
    match demo_args.robot {
        Robot::Irb6640 => irb6640(&demo_args.r, &demo_args.t),
        Robot::KukaR800FixedQ3 => kuka_r800_fixed_q3(&demo_args.r, &demo_args.t),
    }
}

fn display_ik_result(result: (Vec<Vector6<f64>>, Vec<bool>)) {
    if result.0.len() == 0 {
        println!("No solutions");
    }

    for (i, (q, is_ls)) in result.0.into_iter().zip(result.1.into_iter()).enumerate() {
        let ls_display = if is_ls { " (least squares)" } else { "" };
        println!("q{}{ls_display}{q}", i + 1);
    }
}

fn main() {
    let mut robot_map = HashMap::new();

    robot_map.insert("irb-6640", Robot::Irb6640);
    robot_map.insert("kuka-r800-fixed-q3", Robot::KukaR800FixedQ3);

    let mut args = env::args();

    let exe_name = args.next().unwrap();
    let first_arg = args.next();

    if let Some(s) = first_arg {
        if s != "--help" {
            match parse_args(&robot_map) {
                Ok(demo_args) => {
                    display_ik_result(compute_ik(demo_args));
                    return;
                },
                Err(s) => eprintln!("{s}\nRun with \"--help\" for usage information")
            }
        }
    }

    print_usage(&exe_name, robot_map.keys().collect());
}
