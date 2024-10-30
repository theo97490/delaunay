pub mod geometry;
use ffi::DrawableMesh;
use geometry::{load_point_cloud, LoadError, Triangulation2D};

#[cxx::bridge]
pub mod ffi {
    #[namespace = "LibDelaunay"]
    struct Vec3 {
        v: [f64; 3]
    }

    #[namespace = "LibDelaunay"]
    struct Triangle {
        vertices: [usize; 3]
    }

    #[namespace = "LibDelaunay"]
    struct DrawableMesh {
        vertices: Vec<Vec3>,
        triangles: Vec<Triangle>
    }

    #[namespace = "LibDelaunay"]
    extern "Rust" {
        pub fn compute_triangulation_2d(path: String, scale: f64) -> Result<DrawableMesh>;
    }
}

impl From<&geometry::Triangle> for ffi::Triangle{
    fn from(value: &geometry::Triangle) -> Self {
        ffi::Triangle{ vertices: value.vertices }
    }
}

impl From<geometry::Vec3> for ffi::Vec3{
    fn from(value: geometry::Vec3) -> Self {
        ffi::Vec3{ v : [value.x, value.y, value.z ]}
    }
}

impl From<&geometry::TriangleMesh> for ffi::DrawableMesh {
    fn from(tri: &geometry::TriangleMesh) -> Self {
        DrawableMesh {
            vertices: tri.vertices.iter().map(|v| ffi::Vec3::from(v.point)).collect(),
            triangles: tri.triangles.iter().map(ffi::Triangle::from).collect() 
        }
    }
}

fn compute_triangulation_2d<'a>(path: String, scale: f64) -> Result<DrawableMesh, LoadError<'a>> {
    let mut tri = Triangulation2D::default();
    let cloud = load_point_cloud(path.as_str())?;
    for point in cloud {
        tri.add_point(point * scale);
    }

    Ok(DrawableMesh::from(&tri.mesh))
}

