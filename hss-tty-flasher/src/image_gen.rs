use std::fs;
use std::path::Path;
use std::process::Command;

pub fn generate_hss_payload(elf_path: &str) -> std::io::Result<String> {
    let path = Path::new(elf_path);
    let elf_name = path.file_name().unwrap().to_str().unwrap();
    let file_name = path.file_stem().unwrap().to_str().unwrap();
    let dir = path.parent().unwrap_or(Path::new("."));

    // Create the yaml content using the CONFIG template
    let yaml_content = CONFIG.replace("{}", elf_name);
    let yaml_filename = format!("{}-image-conf.yaml", file_name);
    let yaml_path = dir.join(&yaml_filename);
    fs::write(&yaml_path, yaml_content)?;

    let image_filename = format!("{}.img", file_name);
    let image_path = dir.join(&image_filename);

    let output = Command::new("hss-payload-generator")
        .current_dir(dir)
        .arg("-c")
        .arg(&yaml_filename)
        .arg(&image_filename)
        .output()?;

    if !output.status.success() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            "hss-payload-generator command failed",
        ));
    }

    Ok(image_path.to_string_lossy().into_owned())
}

const CONFIG: &str = r#"
set-name: 'PolarFire-SoC-HSS::{}'

hart-entry-points:
  u54_1: '0x1000000000'
  u54_2: '0x1000000000'
  u54_3: '0x1000000000'
  u54_4: '0x1000000000'

payloads:
  {}:
    owner-hart: u54_1
    secondary-hart: u54_2
    secondary-hart: u54_3
    secondary-hart: u54_4
    priv-mode: prv_m
    skip-opensbi: true
"#;
