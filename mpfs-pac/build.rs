use std::path::PathBuf;
use walkdir::WalkDir;

fn get_board_path() -> &'static str {
    if cfg!(feature = "beaglev-fire") {
        "beaglev-fire"
    } else {
        panic!("No board feature selected. Please enable a board feature (e.g., --features beaglev-fire)")
    }
}

fn get_target_board_define() -> &'static str {
    if cfg!(feature = "beaglev-fire") {
        "TARGET_BEAGLEV_FIRE"
    } else {
        panic!("No board feature selected. Please enable a board feature (e.g., --features beaglev-fire)")
    }
}

fn main() {
    println!("cargo:rerun-if-changed=mpfs-platform");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=linker-upper.ld");
    println!("cargo:rerun-if-changed=linker-lower.ld");

    // Skip compilation steps if we're running on docs.rs
    if std::env::var("DOCS_RS").is_ok() {
        // Still generate bindings for documentation
        generate_bindings();
        return;
    }

    // Find all C source files, excluding specific directories
    let c_sources: Vec<PathBuf> = WalkDir::new("mpfs-platform")
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| {
            let path = e.path();
            let is_c_file = path.extension().map_or(false, |ext| ext == "c");
            let path_str = path.to_string_lossy().replace('\\', "/");
            let in_mss = path_str.contains("mpfs-platform/platform/drivers/mss");
            let mss_skip = path_str.contains("mss_usb_device.c");
            let in_fpga_ip = path_str.contains("mpfs-platform/platform/drivers/fpga_ip");
            let fpga_include = path_str.contains("core_gpio");

            is_c_file
                && ((in_fpga_ip && fpga_include)
                    || (in_mss && !mss_skip)
                    || !(in_mss || in_fpga_ip))
        })
        .map(|e| e.path().to_owned())
        .collect();

    // println!("cargo:warning=C sources: {:?}", c_sources);

    // Define assembly sources
    let asm_sources = [
        "mpfs-platform/platform/mpfs_hal/startup_gcc/mss_entry.S",
        "mpfs-platform/platform/mpfs_hal/startup_gcc/mss_utils.S",
        "mpfs-platform/platform/hal/hw_reg_access.S",
    ];

    // Common compiler flags from the makefile
    let mut build = cc::Build::new();
    let board = get_board_path();

    let board_include = format!("mpfs-platform/boards/{}", board);
    let board_config_include = format!("mpfs-platform/boards/{}/platform_config", board);
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
        .define("TARGET_G5_SOC", None) // Used to enable MAC
        .define("MSS_MAC_SIMPLE_TX_QUEUE", None)
        .define(get_target_board_define(), None)
        .define("MSS_USB_OTG_DUAL_ROLE_MODE", None) // Enable USB Host and Device
        .includes(&[
            "mpfs-platform/application",
            "mpfs-platform/platform",
            &board_include,
            &board_config_include,
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
    let linker_script = if cfg!(feature = "upper-memory-layout") {
        "linker-upper.ld"
    } else {
        "linker-lower.ld"
    };
    let linker_script_dst = out_dir.join("linker.ld");
    std::fs::copy(linker_script, &linker_script_dst).unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rustc-link-arg=-Tlinker.ld");
    println!("cargo:rustc-link-arg=--gc-sections");

    generate_bindings();
}

// Extract bindings generation into a separate function
fn generate_bindings() {
    let board = get_board_path();
    let board_include = format!("-Impfs-platform/boards/{}", board);
    let board_config_include = format!("-Impfs-platform/boards/{}/platform_config", board);

    let bindings = bindgen::Builder::default()
        .header("mpfs-platform/wrapper.h")
        .use_core()
        .clang_arg("-xc")
        .clang_args(&[
            "-Impfs-platform",
            "-Impfs-platform/platform",
            "-DTARGET_G5_SOC",
            &format!("-D{}", get_target_board_define()),
            "-DMSS_USB_OTG_DUAL_ROLE_MODE",
            &board_include,
            &board_config_include,
        ])
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
