// Compiles the original PX4 C++ Hysteresis so tests can compare against it.
fn main() {
    let px4 = "..";
    cc::Build::new()
        .cpp(true)
        .include("cpp_ref")
        .include(px4)
        .file(format!("{px4}/hysteresis.cpp"))
        .file("cpp_ref/shim.cpp")
        .compile("cpp_hysteresis");
    println!("cargo:rerun-if-changed={px4}/hysteresis.cpp");
    println!("cargo:rerun-if-changed=cpp_ref/shim.cpp");
}
