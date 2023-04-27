use {
    crate::subproblems::setups::DELTA,
    std::fmt::{ Display, Formatter, Result, Debug },
};

#[derive(Debug, Clone)]
pub enum SolutionSet2<T> {
    Zero,
    One(T),
    Two(T, T),
}

#[derive(Debug, Clone)]
pub enum SolutionSet4<T> {
    Zero,
    One(T),
    Two(T, T),
    Three(T, T, T),
    Four(T, T, T, T),
}

const DELTAS_SIZE_2: usize = 4;
const DELTAS_SIZE_4: usize = 8;

impl<T: Copy> SolutionSet2<T> {
    pub fn expect_one(&self) -> T {
        match self {
            Self::Zero => panic!("Found no soliutions where one was expected"),
            Self::One(s) => *s,
            Self::Two(..) => panic!("Found two solutions where one was expected"),
        }
    }

    pub fn expect_two(&self) -> (T, T) {
        match self {
            Self::Zero => panic!("Found no soliutions where two were expected"),
            Self::One(_) => panic!("Found one solution where two were expected"),
            Self::Two(s1, s2) => (*s1, *s2),
        }
    }

    pub fn get_first(&self) -> T {
        match self {
            Self::Zero => panic!("No solutions"),
            Self::One(s) => *s,
            Self::Two(s, _) => *s,
        }
    }

    pub fn get_all(&self) -> Vec<T> {
        match self {
            Self::Zero => vec![],
            Self::One(s) => vec![*s],
            Self::Two(s1, s2) => vec![*s1, *s2],
        }
    }

    pub fn deltas() -> [(f64, f64); DELTAS_SIZE_2] {
        [
            (-DELTA, 0.0),
            (DELTA, 0.0),
            (0.0, -DELTA),
            (0.0, DELTA),
        ]
    }

    pub fn duplicated(&self) -> Self {
        match &self {
            SolutionSet2::One(s) => SolutionSet2::Two(*s, *s),
            SolutionSet2::Two(s1, s2) => SolutionSet2::Two(*s1, *s2),
            SolutionSet2::Zero => SolutionSet2::Zero,
        }
    }

    pub fn size(&self) -> usize {
        match self {
            SolutionSet2::Zero => 0,
            SolutionSet2::One(_) => 1,
            SolutionSet2::Two(..) => 2,
        }
    }
}

impl<T: Copy + Debug> SolutionSet2<T> {
    pub fn as_csv(&self) -> String {
        let mut results = self.get_all().into_iter().map(|v| format!("{v:?}")).collect::<Vec<String>>();
        results.append(&mut vec![String::new(); 2 - results.len()]);
        results.join(",")
    }

    pub fn from_vec(vec: &Vec<T>) -> Self {
        match vec.len() {
            0 => Self::Zero,
            1 => Self::One(vec[0]),
            2 => Self::Two(vec[0], vec[1]),
            i => panic!("Vector {vec:?} contains too many solutions: {i}")
        }
    }
}

impl<T: Copy> SolutionSet4<T> {
    pub fn expect_one(&self) -> T {
        match self {
            Self::One(s) => *s,
            _ => panic!("Expected one solution"),
        }
    }

    pub fn expect_two(&self) -> (T, T) {
        match self {
            Self::Two(s1, s2) => (*s1, *s2),
            _ => panic!("Expected two solutions"),
        }
    }

    pub fn expect_three(&self) -> (T, T, T) {
        match self {
            Self::Three(s1, s2, s3) => (*s1, *s2, *s3),
            _ => panic!("Expected two solutions"),
        }
    }

    pub fn expect_four(&self) -> (T, T, T, T) {
        match self {
            Self::Four(s1, s2, s3, s4) => (*s1, *s2, *s3, *s4),
            _ => panic!("Expected two solutions"),
        }
    }

    pub fn get_first(&self) -> T {
        match self {
            Self::Zero => panic!("No solutions"),
            Self::One(s) => *s,
            Self::Two(s, ..) => *s,
            Self::Three(s, ..) => *s,
            Self::Four(s, ..) => *s,
        }
    }

    pub fn get_all(&self) -> Vec<T> {
        match self {
            Self::Zero => vec![],
            Self::One(s) => vec![*s],
            Self::Two(s1, s2) => vec![*s1, *s2],
            Self::Three(s1, s2, s3) => vec![*s1, *s2, *s3],
            Self::Four(s1, s2, s3, s4) => vec![*s1, *s2, *s3, *s4],
        }
    }

    pub fn deltas() -> [(f64, f64, f64, f64); DELTAS_SIZE_4] {
        [
            (-DELTA, 0.0, 0.0, 0.0),
            (DELTA, 0.0, 0.0, 0.0),
            (0.0, -DELTA, 0.0, 0.0),
            (0.0, DELTA, 0.0, 0.0),
            (0.0, 0.0, -DELTA, 0.0),
            (0.0, 0.0, DELTA, 0.0),
            (0.0, 0.0, 0.0, -DELTA),
            (0.0, 0.0, 0.0, DELTA),
        ]
    }
}

impl<T: Copy + Debug> SolutionSet4<T> {
    pub fn as_csv(&self) -> String {
        let mut results = self.get_all().into_iter().map(|v| format!("{v:?}")).collect::<Vec<String>>();
        results.append(&mut vec![String::new(); 4 - results.len()]);
        results.join(",")
    }

    pub fn from_vec(vec: &Vec<T>) -> Self {
        match vec.len() {
            0 => Self::Zero,
            1 => Self::One(vec[0]),
            2 => Self::Two(vec[0], vec[1]),
            3 => Self::Three(vec[0], vec[1], vec[2]),
            4 => Self::Four(vec[0], vec[1], vec[2], vec[3]),
            i => panic!("Vector {vec:?} contains too many solutions: {i}")
        }
    }
}

impl<T: Display> Display for SolutionSet2<T> {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            Self::Zero => write!(f, "{{ }}"),
            Self::One(s) => write!(f, "{{ {} }}", s),
            Self::Two(s1, s2) => write!(f, "{{ {} {} }}", s1, s2),
        }
    }
}

impl<T: Display> Display for SolutionSet4<T> {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            Self::Zero => write!(f, "{{ }}"),
            Self::One(s) => write!(f, "{{ {} }}", s),
            Self::Two(s1, s2) => write!(f, "{{ {} {} }}", s1, s2),
            Self::Three(s1, s2, s3) => write!(f, "{{ {} {} {} }}", s1, s2, s3),
            Self::Four(s1, s2, s3, s4) => write!(f, "{{ {} {} {} {} }}", s1, s2, s3, s4),
        }
    }
}
