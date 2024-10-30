use std::{fmt::Display, ops};
use overload::overload;

#[derive(Debug, Default, Clone, Copy)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

#[macro_export]
macro_rules! vec3 {
    [] => { Vec3::default() };
    [$x:expr] => {
        Vec3::x($x)
    }; 
    [$x:expr, $y:expr] => {
        Vec3::xy($x, $y)
    };
    [$x:expr, $y:expr, $z:expr] => {
        Vec3::xyz($x, $y, $z)
    }; 
}
pub use vec3;

impl Vec3 {
    pub fn x(x: f64) -> Vec3 { Vec3 {x,y: 0.,z:0.} }
    pub fn xy(x: f64, y: f64) -> Vec3 { Vec3{x,y,z:0.} }
    pub fn xyz(x: f64, y: f64, z: f64) -> Vec3 { Vec3{x,y,z} }
}

impl Display for Vec3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}, {}]", self.x, self.y, self.z)
    }
}

// Using the overload crate, we could use derive_more to completely remove the redefinition
// add
overload!((a: ?Vec3) + (b: ?Vec3) -> Vec3 { vec3![a.x + b.x, a.y + b.y, a.z + b.z] });
overload!((a: &mut Vec3) += (b: ?Vec3) { *a = *a + b });
// sub
overload!((a: ?Vec3) - (b: ?Vec3) -> Vec3 { vec3![a.x - b.x, a.y - b.y, a.z - b.z] });
overload!((a: &mut Vec3) -= (b: ?Vec3) { *a = *a - b });
// mul
overload!((a: ?Vec3) * (b: ?Vec3) -> Vec3 { vec3![a.x * b.x, a.y * b.y, a.z * b.z] });
overload!((a: &mut Vec3) *= (b: ?Vec3) { *a = *a * b });
// mul scalar
overload!((a: ?Vec3) * (b: ?f64) -> Vec3 { vec3![a.x * b, a.y * b, a.z * b] });
overload!((a: &mut Vec3) *= (b: ?f64) { *a = *a * b });
// div
overload!((a: ?Vec3) / (b: ?Vec3) -> Vec3 { vec3![a.x / b.x, a.y / b.y, a.z / b.z] });
overload!((a: &mut Vec3) /= (b: ?Vec3) { *a = *a / b });
