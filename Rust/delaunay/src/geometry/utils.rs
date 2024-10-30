use std::mem::swap;

use super::{Triangle, TriangleMesh, Vec3};

#[derive(Debug, PartialEq, PartialOrd)]
pub enum TriOrient {
    CW = -1,
    Flat = 0,
    CCW = 1
}

pub fn orient(tri: [Vec3; 3]) -> TriOrient {
    let u = tri[1] - tri[0]; 
    let v = tri[2] - tri[0]; 

    // Ensure a good estimation of the zero value
    let orientation = u.x * v.y - u.y * v.x;
    let epsilon = f64::EPSILON * f64::abs(orientation) * 16.; 

    if orientation > -epsilon && orientation < epsilon {
        return TriOrient::Flat; 
    }

    if orientation > 0. {TriOrient::CCW} else {TriOrient::CW}
}

// TODO: Delete this 
pub fn to_ccw(mesh: &TriangleMesh, mut tri: Triangle) -> Triangle {
    let vertices = mesh.get_vertices(&tri);
    let tri_orient = orient(vertices);

    if tri_orient == TriOrient::CW {
        tri.vertices.swap(0, 1);
        // tri.adjacent.swap(0, 1);
    }
    
    tri
} 
