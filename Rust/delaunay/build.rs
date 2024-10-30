fn main() {
    let rs_files = [
        "src/libdelaunay.rs",
        "src/geometry/point.rs"       
    ];

    cxx_build::bridge(rs_files[0])
        .std("c++14")
        .compile("libdelaunay");

    println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=src/blobstore.cc");
    println!("cargo:rerun-if-changed=include/blobstore.h");
}
