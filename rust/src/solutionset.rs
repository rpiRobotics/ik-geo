use std::fmt::{ Display, Formatter, Result };

#[derive(Debug, Clone)]
pub enum SolutionSet2<T> {
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

impl<T> SolutionSet2<T> where T: Copy {
    pub fn expect_one(&self) -> T {
        match self {
            Self::One(s) => *s,
            Self::Two(..) => panic!("Found two solutions where one was expected"),
        }
    }

    pub fn expect_two(&self) -> (T, T) {
        match self {
            Self::One(_) => panic!("Found one solution where two were expected"),
            Self::Two(s1, s2) => (*s1, *s2),
        }
    }

    pub fn get_first(&self) -> T {
        match self {
            Self::One(s) => *s,
            Self::Two(s, _) => *s,
        }
    }

    pub fn get_all(&self) -> Vec<T> {
        match self {
            Self::One(s) => vec![*s],
            Self::Two(s1, s2) => vec![*s1, *s2],
        }
    }
}

impl<T> SolutionSet4<T> where T: Copy {
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
}

impl<T: Display> Display for SolutionSet2<T> {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
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
