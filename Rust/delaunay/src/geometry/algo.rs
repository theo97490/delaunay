use std::borrow::Borrow;

use crate::geometry::{orient, TriOrient};

use super::{TriIt, Triangle, TriangleMesh, Vec3};

impl TriangleMesh {
    // Expensive check
    pub fn assert_mesh_validity(&self, orient_test: bool) {
        let vlen = self.vertices.len();
        if vlen == 0 { return };

        // Test infinite point if its incident tri is valid
        let v_range = if self.is_tri_id_valid(self.vertices[0].incident_tri)
            { 0..vlen } else { 1..vlen };

        for v in v_range {
            // Connected triangle exist and contains the point
            let incident_tri_id = self.vertices[v].incident_tri;
            debug_assert!(self.is_tri_id_valid(incident_tri_id));
            let incident_tri = &self.triangles[incident_tri_id];
            debug_assert!(incident_tri.id_of_v(v).is_valid())
        }

        for tri_id in 0..self.triangles.len() {
            // Vertices are unique
            let tri = &self.triangles[tri_id];
            for i in TriIt::range() {
                let v = tri.vertices[i];
                debug_assert!(self.is_v_id_valid(v));
                debug_assert!(tri.vertices[i.next()] != v);
                debug_assert!(tri.vertices[i.prev()] != v);
            }

            // adjacent triangles <=> triangle connectivity
            for i in TriIt::range() {
                let adj_tri_id = tri.adjacent[i];
                if self.is_tri_id_valid(adj_tri_id) {
                    let adj_tri = &self.triangles[adj_tri_id];
                    let adj_edge_id = adj_tri.id_of_t(tri_id);

                    let edge = tri.get_edge(i);
                    let adj_edge = adj_tri.get_edge(adj_edge_id);

                    debug_assert!(edge.0 != edge.1 && adj_edge.0 != adj_edge.1);
                    let same_edge = {
                        (edge.0 == adj_edge.0 || edge.0 == adj_edge.1)
                        && (edge.1 == adj_edge.0 || edge.1 == adj_edge.1)
                    };

                    debug_assert!(same_edge);
                }
            }

            // orientation test if triangle is not infinite
            if orient_test && !tri.is_infinite() {
                debug_assert!(orient(self.get_vertices(tri)) >= TriOrient::Flat)
            }
        }
    }

    pub fn split_face(&mut self, tri_id: usize, point: Vec3) -> (usize, [usize; 3]) {
        let old_tri = self.triangles[tri_id].clone();
        let point_id = self.add_point(point);

        let new_tris = [tri_id, self.triangles.len(), self.triangles.len() + 1];
        self.triangles.resize(self.triangles.len() + 2, Triangle::default());

        for i in TriIt::range() {
            let current_tri_id = new_tris[i];

            let tri_adj_id = old_tri.adjacent[i];
            if self.is_tri_id_valid(tri_adj_id) {
                let tri_adj = &mut self.triangles[tri_adj_id];
                let o_edge = tri_adj.id_of_t(tri_id);
                tri_adj.adjacent[o_edge] = current_tri_id;
            }
            
            let tri = &mut self.triangles[current_tri_id];
            let edge = old_tri.get_edge(i);
            self.vertices[old_tri.vertices[i.next()]].incident_tri = current_tri_id;
            tri.vertices = [point_id, edge.0, edge.1];
            tri.adjacent = [old_tri.adjacent[i], new_tris[i.next()], new_tris[i.prev()]];
        }

        self.vertices[point_id].incident_tri = tri_id;
        self.assert_mesh_validity(true);
        (point_id, new_tris)
    }

    pub fn split_edge(&mut self, tri_id: usize, edge_id: TriIt, point: Vec3) -> usize {
        todo!()
    }

    pub fn edge_flip(&mut self, tri_id_0: usize, edge_id: TriIt) {
        let tri_0 = self.triangles[tri_id_0].clone();
        let tri_id_1 = tri_0.adjacent[edge_id];
        let edge = tri_0.get_edge(edge_id);
        assert!(self.is_tri_id_valid(tri_id_1));
        let tri_1 = self.triangles[tri_id_1].clone();
        let o_edge_id = tri_1.id_of_t(tri_id_0);

        let tri_0_e0 = edge_id.next();
        let tri_0_e1 = edge_id.prev();
        let tri_1_e0 = tri_1.id_of_v(edge.0);
        let tri_1_e1 = tri_1.id_of_v(edge.1);

        let mut_tri_0 = &mut self.triangles[tri_id_0];
        mut_tri_0.vertices[tri_0_e1] = tri_1.vertices[o_edge_id];
        mut_tri_0.adjacent[edge_id] = tri_1.adjacent[tri_1_e1];
        mut_tri_0.adjacent[tri_0_e0] = tri_id_1;

        let mut_tri_1 = &mut self.triangles[tri_id_1];
        mut_tri_1.vertices[tri_1_e0] = tri_0.vertices[edge_id];
        mut_tri_1.adjacent[o_edge_id] = tri_0.adjacent[tri_0_e0];
        mut_tri_1.adjacent[tri_1_e1] = tri_id_0;
    
        self.vertices[tri_0.vertices[edge_id]].incident_tri = tri_id_0;
        self.vertices[tri_0.vertices[tri_0_e0]].incident_tri = tri_id_0;
        self.vertices[tri_0.vertices[tri_0_e1]].incident_tri = tri_id_1;
        self.vertices[tri_1.vertices[o_edge_id]].incident_tri = tri_id_1;

        let adj_tri_0 = tri_1.adjacent[tri_1_e1];
        let adj_tri_1 = tri_0.adjacent[tri_0_e0];

        if self.is_tri_id_valid(adj_tri_0) {
            let adj_tri = &mut self.triangles[adj_tri_0];
            let adj_edge_id = adj_tri.id_of_t(tri_id_1);
            adj_tri.adjacent[adj_edge_id] = tri_id_0;
        }

        if self.is_tri_id_valid(adj_tri_1) {
            let adj_tri = &mut self.triangles[adj_tri_1];
            let adj_edge_id = adj_tri.id_of_t(tri_id_0);
            adj_tri.adjacent[adj_edge_id] = tri_id_1;
        }

        self.assert_mesh_validity(true);
    }
}

