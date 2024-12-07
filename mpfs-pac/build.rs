use std::path::PathBuf;
use walkdir::WalkDir;

fn main() {
    // Find all C source files, excluding specific directories
    let c_sources: Vec<PathBuf> = WalkDir::new("mpfs-platform")
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| {
            let path = e.path();
            let is_c_file = path.extension().map_or(false, |ext| ext == "c");
            let path_str = path.to_string_lossy().replace('\\', "/");
            let in_mss = path_str.contains("mpfs-platform/platform/drivers/mss");
            let in_allowed_mss = path_str.contains("mss_uart")
                || path_str.contains("mss_gpio")
                || path_str.contains("mss_timer");

            is_c_file && (!in_mss || in_allowed_mss)
        })
        .map(|e| e.path().to_owned())
        .collect();

    // Define assembly sources
    let asm_sources = [
        "mpfs-platform/platform/mpfs_hal/startup_gcc/mss_entry.S",
        "mpfs-platform/platform/mpfs_hal/startup_gcc/mss_utils.S",
        "mpfs-platform/platform/hal/hw_reg_access.S",
    ];

    // Common compiler flags from the makefile
    let mut build = cc::Build::new();
    build
        .flag("-march=rv64gc")
        .flag("-mabi=lp64d")
        .flag("-mcmodel=medany")
        .flag("-msmall-data-limit=8")
        .flag("-mstrict-align")
        .flag("-mno-save-restore")
        .flag("-Os")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .flag("-fsigned-char")
        .flag("-g")
        .define("NDEBUG", None)
        .includes(&[
            "mpfs-platform/application",
            "mpfs-platform/platform",
            "mpfs-platform/boards/beaglev-fire",
            "mpfs-platform/boards/beaglev-fire/platform_config",
        ]);

    // Compile C sources
    let mut c_build = build.clone();
    c_build
        .compiler("riscv64-unknown-elf-gcc")
        .std("gnu11")
        .flag("-Wstrict-prototypes")
        .flag("-Wbad-function-cast")
        .flag("-Wno-sign-compare")
        .flag("-Wno-pointer-compare")
        .flag("-Wno-int-to-pointer-cast")
        .flag("-Wno-pointer-to-int-cast")
        .flag("-Wno-unused-parameter")
        .flag("-Wno-unused-variable")
        .flag("-Wno-unused-but-set-variable")
        .files(c_sources);

    // Compile assembly sources
    let mut asm_build = build.clone();
    asm_build
        .compiler("riscv64-unknown-elf-gcc")
        .files(asm_sources)
        .flag("-x")
        .flag("assembler-with-cpp");

    // Perform the compilation
    c_build.compile("platform_c");
    asm_build.compile("platform_asm");

    // Linker
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let linker_script = "linker.ld";
    let linker_script_dst = out_dir.join("linker.ld");
    std::fs::copy(linker_script, &linker_script_dst).unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rustc-link-arg=-Tlinker.ld");
    println!("cargo:rustc-link-arg=--gc-sections");

    // Generate bindings
    let bindings = bindgen::Builder::default()
        .header("mpfs-platform/wrapper.h")
        .use_core()
        .clang_arg("-xc")
        .clang_args(&[
            "-Impfs-platform",
            "-Impfs-platform/platform",
            "-Impfs-platform/boards/beaglev-fire",
            "-Impfs-platform/boards/beaglev-fire/platform_config",
        ])
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to an output file
    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    println!("cargo:rerun-if-changed=mpfs-platform");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=linker.ld");
}
