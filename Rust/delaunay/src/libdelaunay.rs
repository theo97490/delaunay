pub mod geometry;
use ffi::DrawableMesh;
use geometry::{load_point_cloud, LoadError, Triangulation, Triangulation2D};
use rand::seq::IndexedRandom;


enum TriEnum {
    Naive(Triangulation2D),
    Delaunay(Triangulation2D)
}

struct Configuration {
    cloud: Vec<geometry::Vec3>,
    scale: f64,
    elevate: bool,
    vertex: usize,
    triangulation: TriEnum
}

static mut GLOBAL_CONFIG: Option<Configuration> = None;

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
    #[derive(Default)]
    struct DrawableMesh {
        vertices: Vec<Vec3>,
        triangles: Vec<Triangle>
    }

    #[namespace = "LibDelaunay"]
    extern "Rust" {
        pub fn compute_triangulation_2d(path: String, scale: f64, elevate: bool) -> Result<DrawableMesh>;
        pub fn start_triangulation_2d(path: String, scale: f64, elevate: bool, init_vertices: usize) -> Result<DrawableMesh>;
        pub fn compute_next() -> DrawableMesh;
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

fn compute_triangulation_2d<'a>(path: String, scale: f64, elevate: bool) -> Result<DrawableMesh, LoadError<'a>> {
    let mut tri = Triangulation2D::default();
    let cloud = load_point_cloud(path.as_str())?;
    for mut point in cloud {
        point *= scale;
        if !elevate { point.z = 0.; }
        tri.add_point(point);
    }

    Ok(DrawableMesh::from(&tri.mesh))
}

pub fn start_triangulation_2d<'a>(path: String, scale: f64, elevate: bool, init_vertices: usize) -> Result<DrawableMesh, LoadError<'a>> {
    start_triangulation(TriEnum::Naive(Triangulation2D::default()), path, scale, elevate, init_vertices)
}

fn start_triangulation<'a>(triangulation: TriEnum, path: String, scale: f64, elevate: bool, init_vertices: usize) -> Result<DrawableMesh, LoadError<'a>> {
    let cloud = load_point_cloud(path.as_str())?;
    unsafe { 
        GLOBAL_CONFIG = Some(Configuration {
            cloud, scale, elevate, vertex: init_vertices, triangulation
        });
    }

    compute_next();
    todo!()
}

fn compute_next() -> DrawableMesh {
    let mut opt : &mut Option<Configuration>;

    // Assuming all operations are single threaded
    unsafe { opt = &mut GLOBAL_CONFIG; }

    if let Some(config) = &mut opt {
        if config.vertex >= config.cloud.len()
            { return DrawableMesh::default(); }

        let mut point = config.cloud[config.vertex];
        point *= config.scale;
        if !config.elevate { point.z = 0.; }

        config.vertex += 1;

        match &mut config.triangulation {
                TriEnum::Naive(tri) => { tri.add_point(point); DrawableMesh::from(&tri.mesh) },
                TriEnum::Delaunay(tri) => { tri.add_point(point); DrawableMesh::from(&tri.mesh) },
        };
    }

    DrawableMesh::default()
}

