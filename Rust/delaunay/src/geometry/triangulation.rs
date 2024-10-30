use std::collections::{HashMap, HashSet};

use rand::{random, seq::IndexedRandom, Rng};

use crate::geometry::{orient, FaceIt, TriIt, Triangle};

use super::{to_ccw, TriOrient, TriangleMesh, Vec3};

type Edge = (usize, usize);

pub struct SimpleTriangulation<'a> {
    mesh: &'a TriangleMesh,
    edge_to_tri: HashMap<Edge, usize>
}

#[derive(Default)]
pub struct Triangulation2D {
    pub mesh: TriangleMesh
}

impl<'a> SimpleTriangulation<'a> {
    pub fn new(m: &'a TriangleMesh) -> Self { Self { mesh: m, edge_to_tri: HashMap::new() } }
    // TODO: Recoder le tp1 plus tard
}

impl Triangulation2D {
    pub fn add_point(&mut self, point: Vec3) -> usize {
        if self.mesh.vertices.len() < 4 {
            let point_id = self.mesh.add_point(point);
            if point_id >= 3 {
                self.init_first_triangle();
            }

            return point_id
        }

        let (tri_id, edge_id, tri_orient) = self.find_nearest_tri(point);

        match tri_orient {
            TriOrient::CW => self.add_point_outside_hull(tri_id, edge_id, point),
            TriOrient::Flat => self.mesh.split_edge(tri_id, edge_id, point),
            TriOrient::CCW => self.mesh.split_face(tri_id, point).0
        }
    }

    fn add_point_outside_hull(&mut self, tri_id: usize, edge_id: TriIt, point: Vec3) -> usize {
        let tri = &self.mesh.triangles[tri_id];
        let hull_tri_id = tri.adjacent[edge_id];
        assert!(self.mesh.is_tri_id_valid(hull_tri_id));
        let (point_id, faces_id) = self.mesh.split_face(hull_tri_id, point);
        
        let mut result : Option<(FaceIt, FaceIt)> = None;

        for i in TriIt::range() {
            let h1 = FaceIt::new(&self.mesh, faces_id[i], TriangleMesh::INFINITE);
            if !h1.id_of_v(TriangleMesh::INFINITE).is_valid() { continue; }
            if !h1.id_of_v(point_id).is_valid() { continue; }

            if let Some(h0) = h1.prev() {
                if !h0.id_of_v(TriangleMesh::INFINITE).is_valid() { continue; }
                if !h0.id_of_v(point_id).is_valid() { continue; }
                result = Some((h0, h1));
                break;
            }
        }

        assert!(result.is_some());
        let (hull_0, hull_1) = result.unwrap();
        let mut tris_to_flip = Vec::<(usize, TriIt)>::with_capacity(10); 
        
        let mut get_faces_to_flip = |hull_start: FaceIt, sign: bool| {
            let mut face_it = hull_start.step(sign).expect("Expected at least one neighboring face to vertex");
            
            loop {
                let inf_id = face_it.id_of_v(TriangleMesh::INFINITE);
                let vertices = [
                    self.mesh.get_v(face_it.get_tri(), inf_id.next().id()),
                    self.mesh.get_v(face_it.get_tri(), inf_id.prev().id()),
                    point
                ];

                if orient(vertices) < TriOrient::Flat { break; }
                tris_to_flip.push((face_it.get_tri_id(), inf_id.step(!sign)));
                face_it = face_it.step(sign).unwrap();

                if face_it == hull_start { break; }
            }
        };

        get_faces_to_flip(hull_0, false);
        get_faces_to_flip(hull_1, true);

        for (tri_id, edge_id) in tris_to_flip {
            self.mesh.edge_flip(tri_id, edge_id);
        }

        self.mesh.assert_mesh_validity(true);
        point_id
    }

    fn find_nearest_tri(&self, point: Vec3) -> (usize, TriIt, TriOrient) {
        let mut tri_id : usize;
        let mut tri: &Triangle;
        let mut rng = rand::thread_rng();

        loop {
            tri_id = rng.gen_range(0..self.mesh.triangles.len());
            tri = &self.mesh.triangles[tri_id];
            if !tri.is_infinite() { break; }
        }
        
        let vertices = self.mesh.get_vertices(tri);
        
        let mut min_edge_id = TriIt::from(0);
        let mut min_orient = TriOrient::CCW;

        for id in TriIt::range() {
            let tri = [
                vertices[id.next()],
                vertices[id.prev()],
                point
            ];
            let tri_orient = orient(tri);
            if tri_orient <= min_orient {
                min_edge_id = id;
                min_orient = tri_orient;
            }
        }

        if min_orient >= TriOrient::Flat {
            return (tri_id, min_edge_id, min_orient)
        }

        while min_orient == TriOrient::CW {
            let tri_adj_id = tri.adjacent[min_edge_id]; 
            if !self.mesh.is_tri_id_valid(tri_adj_id) { break; }

            let tri_adj = &self.mesh.triangles[tri_adj_id];
            if tri_adj.is_infinite() { break; }

            let edge_id = tri_adj.id_of_t(tri_id);
            let vertices = self.mesh.get_vertices(tri_adj);

            min_orient = orient([vertices[edge_id.prev()], vertices[edge_id], point]);
            min_edge_id = edge_id.next();

            let orient_1 = orient([vertices[edge_id], vertices[edge_id.next()], point]);
            if orient_1 < min_orient {
                min_orient = orient_1;
                min_edge_id = edge_id.prev();
            }

            tri_id = tri_adj_id;
            tri = tri_adj;
        }

        (tri_id, min_edge_id, min_orient)
    }

    fn init_first_triangle(&mut self) {
        let mut tri = Triangle { 
            vertices: [ 1, 2, 3 ],
            adjacent: [ 1, 2, 3 ]
        };

        // TODO: Inline function and remove to_ccw
        tri = to_ccw(&self.mesh, tri);
        assert!(orient(self.mesh.get_vertices(&tri)) >= TriOrient::Flat);
        self.mesh.triangles.push(tri.clone());
    
        for i in TriIt::range() {
            self.mesh.triangles.push(Triangle{
                vertices: [ TriangleMesh::INFINITE, tri.vertices[i.prev()], tri.vertices[i.next()] ],
                adjacent: [ 0, 1 + i.prev().id(), 1 + i.next().id() ]
            });
        }

        self.mesh.vertices[TriangleMesh::INFINITE].incident_tri = 1;
        self.mesh.vertices[1].incident_tri = 0;
        self.mesh.vertices[2].incident_tri = 0;
        self.mesh.vertices[3].incident_tri = 0;

        self.mesh.assert_mesh_validity(true);
    }
}
