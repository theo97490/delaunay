use std::fmt::Display;
use std::io::{self, BufRead};
use std::num::{ParseFloatError, ParseIntError};
use std::{fs::File, io::BufReader};
use super::{TriangleMesh, Vec3};

#[derive(Debug)]
pub enum LoadError<'a> {
    IO(io::Error),
    ParseInt(std::num::ParseIntError),
    ParseFloat(ParseFloatError),
    Parse(&'a str)
}

impl<'a> Display for LoadError<'a> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IO(e) => write!(f, "{}", e),
            Self::ParseInt(e) => write!(f, "{}", e),
            Self::ParseFloat(e) => write!(f, "{}", e),
            Self::Parse(e) => write!(f, "{}", e),
        }
    }
}
impl<'a> From<io::Error> for LoadError<'a> {
    fn from(v: io::Error) -> Self {
        Self::IO(v)
    }
}

impl<'a> From<ParseIntError> for LoadError<'a> {
    fn from(v: ParseIntError) -> Self {
        Self::ParseInt(v)
    }
}

impl<'a> From<ParseFloatError> for LoadError<'a> {
    fn from(v: ParseFloatError) -> Self {
        Self::ParseFloat(v)
    }
}

impl<'a> From<&'a str> for LoadError<'a> {
    fn from(v: &'a str) -> Self {
        Self::Parse(v)
    }
}

pub fn load_off_mesh(path: &str) -> TriangleMesh {
    TriangleMesh::default()
}

pub fn load_point_cloud<'a>(path: &str) -> Result<Vec<Vec3>, LoadError<'a>> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut line = String::new();
    let mut vec : Vec<Vec3> = Vec::new();

    reader.read_line(&mut line)?;

    let v_count : usize = line.trim().parse()?;
    vec.reserve(v_count);
    
    for _ in 0..v_count {
        line.clear();
        reader.read_line(&mut line)?;
        let mut parts = line.split_whitespace();
        let x : f64 = parts.next().ok_or("Missing x in parsed vec3")?.trim().parse()?;
        let y : f64 = parts.next().ok_or("Missing y in parsed vec3")?.trim().parse()?;
        let z : f64 = parts.next().ok_or("Missing z in parsed vec3")?.trim().parse()?;
        vec.push(Vec3::xyz(x,y,z));
    }

    Ok(vec)
}
