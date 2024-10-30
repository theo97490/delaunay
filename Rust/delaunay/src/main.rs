mod geometry;

use geometry::*;

// fn main() {
//
//     let mut tri = Triangulation2D::default();
//     let mesh = &mut tri.mesh;
//
//     mesh.vertices.extend([
//         Vertex::new(vec3![0., 1., 3.], 0),
//         Vertex::new(vec3![0., 1., 3.], 0),
//         Vertex::new(vec3![0., 1., 3.], 0),
//         Vertex::new(vec3![0., 1., 3.], 1),
//     ]);
//
//     let n = TriangleMesh::INVALID_TRIANGLE;
//     mesh.triangles.extend([
//         Triangle::new([1,2,3], [n,1,2]),
//         Triangle::new([1,3,4], [n,2,0]),
//         Triangle::new([1,4,2], [n,0,1]),
//     ]);
//
//     mesh.assert_mesh_validity(false);
//     mesh.split_face(0, vec3![]);
//     // mesh.edge_flip(0, TriIt::from(1));
//     // let tuple = tri.find_nearest_tri(vec3![-10.,-5.,0.]);
//
//     let begin = tri.mesh.face_it(1);
//     let mut it = begin;
//  
//     loop {
//         println!("{}", it.get_tri_id());
//         it = it.next().expect("Faces should be circular");
//         if it == begin { break; } 
//     }
// }
fn main() {

    let mut tri = Triangulation2D::default();

    tri.add_point(vec3![0.,0.,0.]);
    tri.add_point(vec3![5.,5.,0.]);
    tri.add_point(vec3![5.,0.,0.]);
    tri.add_point(vec3![0.,5.,0.]);

    let begin = tri.mesh.face_it(0);
    let mut it = begin;
 
    loop {
        println!("{}", it.get_tri_id());
        it = it.next().expect("Faces should be circular");
        if it == begin { break; } 
    }
}
