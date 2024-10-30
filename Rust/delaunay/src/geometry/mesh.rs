use std::{ops::{Deref, DerefMut}, vec};
use super::{Vec3, LocalId, vec3};

pub type TriIt = LocalId<3>; 

#[derive(Default, Debug, Clone)]
pub struct Triangle {
    pub vertices: [usize; 3],
    pub adjacent: [usize; 3],
}

#[derive(Default, Debug, Clone)]
pub struct Vertex {
    pub point: Vec3,
    pub incident_tri: usize
}

#[derive(Debug)]
pub struct TriangleMesh {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<Triangle>
}

#[derive(Clone, Copy)]
pub struct FaceIt<'a> {
    mesh: &'a TriangleMesh,
    triangle: usize,
    vertex: usize
}

impl Vertex {
    pub fn new(p: Vec3, tri: usize) -> Self { Self {point: p, incident_tri: tri} }
}

impl Triangle {
    pub fn new(v: [usize;3], tri: [usize;3]) -> Self { Self { vertices: v, adjacent: tri} }
    pub fn id_of_v(&self, v: usize) -> TriIt {
        for i in 0..3 {
            if self.vertices[i] == v {
                return TriIt::from(i)
            } 
        }

        TriIt::make_invalid()
    } 

    pub fn id_of_t(&self, t: usize) -> TriIt{
        for i in 0..3 {
            if self.adjacent[i] == t {
                return TriIt::from(i)
            } 
        }

        TriIt::make_invalid()
    }

    pub fn get_edge(&self, i: TriIt) -> (usize, usize) {
        (self.vertices[i.next()], self.vertices[i.prev()])
    }

    // pub fn find_edge(&self, edge: (usize, usize)) -> TriIt {
    //     for i in TriIt::range() {
    //         self.vertices[]
    //     }
    // }

    pub fn is_infinite(&self) -> bool {
        self.id_of_v(TriangleMesh::INFINITE).is_valid()
    }
}

impl Default for TriangleMesh {
    fn default() -> Self {
        let infinite_point = Vertex{point: vec3![0.,0.,f64::INFINITY], incident_tri: usize::MAX };  
        Self {
            triangles: Vec::new(),
            vertices: vec![infinite_point]
        }
    }
}

impl TriangleMesh {
    pub const INFINITE: usize = 0;
    pub const INVALID_VERTEX: usize = usize::MAX;   
    pub const INVALID_TRIANGLE: usize = usize::MAX;

    pub fn is_v_id_valid(&self, v: usize) -> bool { v < self.vertices.len() }
    pub fn is_tri_id_valid(&self, t: usize) -> bool { t < self.triangles.len() }

    pub fn get_v(&self, tri: &Triangle, i: usize) -> Vec3 { self.vertices[tri.vertices[i]].point }
    pub fn get_vertices(&self, tri: &Triangle) -> [Vec3; 3] {
        [
            self.vertices[tri.vertices[0]].point,
            self.vertices[tri.vertices[1]].point,
            self.vertices[tri.vertices[2]].point,
        ]
    }

    pub fn get_tri(&self, tri: &Triangle, v: TriIt) -> &Triangle {
        &self.triangles[tri.adjacent[v.id()]]
    }

    pub fn face_it(&self, v: usize) -> FaceIt{
        let tri = self.vertices[v].incident_tri;
        assert!(self.is_tri_id_valid(tri), "FaceIt failed : Associated face to vertex wasn't valid or didn't exist");
        FaceIt::new(self, tri, v)
    }

    pub fn add_point(&mut self, point: Vec3) -> usize {
        let id = self.vertices.len();
        self.vertices.push(Vertex {point, incident_tri: usize::MAX});
        id
    }
}

impl<'a> FaceIt<'a> {
    pub fn new(mesh: &'a TriangleMesh, tri: usize, v: usize) -> Self { Self{mesh, triangle:tri, vertex:v} }

    pub fn prev(self) -> Option<Self> {
        let tri = &self.mesh.triangles[self.triangle];
        let v_id = tri.id_of_v(self.vertex);
        let o_tri = tri.adjacent[v_id.prev().id()];
        if !self.mesh.is_tri_id_valid(o_tri) {
            return None
        }

        Some(FaceIt::new(self.mesh, o_tri, self.vertex))
    }

    pub fn next(self) -> Option<Self> {
        let tri = &self.mesh.triangles[self.triangle];
        let v_id = tri.id_of_v(self.vertex);
        let o_tri = tri.adjacent[v_id.next().id()];
        if !self.mesh.is_tri_id_valid(o_tri) {
            return None
        }

        Some(FaceIt::new(self.mesh, o_tri, self.vertex))
    }

    pub fn step(self, sign: bool) -> Option<Self> {
        if sign { self.next() }
        else { self.prev() }
    }
    

    pub fn get_mesh(&self) -> &TriangleMesh { self.mesh }
    pub fn get_tri_id(&self) -> usize { self.triangle }
    pub fn get_tri(&self) -> &Triangle { &self.mesh.triangles[self.triangle] }
    // pub fn get_tri_mut(&self) -> &mut Triangle { &mut self.mesh.triangles[self.triangle] }
}

impl<'a> Deref for FaceIt<'a> {
    type Target = Triangle;

    fn deref(&self) -> &Self::Target {
        self.get_tri()
    }
}

// impl<'a> DerefMut for FaceIt<'a> {
//     fn deref_mut(&mut self) -> &mut Self::Target {
//         self.get_tri_mut()
//     }
// }

impl<'a> PartialEq for FaceIt<'a>{
    fn eq(&self, other: &Self) -> bool {
        std::ptr::eq(self.mesh, other.mesh) && self.triangle == other.triangle && self.vertex == other.vertex
    }
}
