use {
    std::{ env, collections::HashMap },
    linear_subproblem_solutions_rust::inverse_kinematics::hardcoded::{ irb6640, kuka_r800_fixed_q3, rrc_fixed_q6, ur5, three_parallel_bot, two_parallel_bot, spherical_bot, yumi_fixed_q3 },
    nalgebra::{ Matrix3, Vector3, Vector6 },
};

#[derive(PartialEq, Eq, Hash, Clone, Copy)]
enum Robot {
    Irb6640,
    KukaR800FixedQ3,
    RrcFixedQ6,
    YumiFixedQ3,
    Ur5,
    ThreeParallelBot,
    TwoParallelBot,
    SphericalBot,
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

fn parse_args(robot_lookup: &HashMap<&str, Robot>) -> Result<DemoArgs, String> {
    fn extract_float<T: Iterator<Item = String>>(iter: &mut T) -> Result<f64, String> {
        let raw = iter.next().ok_or("Not enough arguments".to_owned())?;
        raw.parse().or(Err(format!("Malformed float {raw:?}")))
    }

    let mut args = env::args().skip(1);

    let robot = args.next().ok_or("No robot specified".to_owned())?;
    let robot = *robot_lookup.get(&robot[..]).ok_or(format!("Invalid robot {robot:?}"))?;

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

fn compute_ik(demo_args: DemoArgs, function_lookup: &HashMap<Robot, fn(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>)>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    function_lookup.get(&demo_args.robot).unwrap()(&demo_args.r, &demo_args.t)
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

macro_rules! create_lookups {
    ($($s:literal, $p:expr, $f:ident),*,) => {{
        let mut robot_lookup = HashMap::new();
        let mut function_lookup: HashMap<Robot, fn(r: &Matrix3<f64>, t: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>)> = HashMap::new();

        $(
            robot_lookup.insert($s, $p);
            function_lookup.insert($p, $f);
        )*

        (robot_lookup, function_lookup)
    }};
}

fn main() {
    let (robot_lookup, function_lookup) = create_lookups!(
        "irb-6640", Robot::Irb6640, irb6640,
        "kuka-r800-fixed-q3", Robot::KukaR800FixedQ3, kuka_r800_fixed_q3,
        "rrc-fixed-q6", Robot::RrcFixedQ6, rrc_fixed_q6,
        "yumi-fixed-q3", Robot::YumiFixedQ3, yumi_fixed_q3,
        "ur5", Robot::Ur5, ur5,
        "three-parallel-bot", Robot::ThreeParallelBot, three_parallel_bot,
        "two-parallel-bot", Robot::TwoParallelBot, two_parallel_bot,
        "spherical-bot", Robot::SphericalBot, spherical_bot,
    );

    let mut args = env::args();

    let exe_name = args.next().unwrap();
    let first_arg = args.next();

    if let Some(s) = first_arg {
        if s != "--help" {
            match parse_args(&robot_lookup) {
                Ok(demo_args) => display_ik_result(compute_ik(demo_args, &function_lookup)),
                Err(s) => eprintln!("ERROR: {s}\nRun with \"--help\" for usage information")
            }

            return;
        }
    }

    print_usage(&exe_name, robot_lookup.keys().collect());
}
