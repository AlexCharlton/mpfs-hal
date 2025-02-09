use std::fs;
use std::path::Path;
use std::process::Command;

pub fn generate_hss_payload(elf_path: &str) -> std::io::Result<String> {
    let path = Path::new(elf_path);
    let elf_name = path.file_name().unwrap().to_str().unwrap();
    let file_name = path.file_stem().unwrap().to_str().unwrap();
    let dir = path.parent().unwrap_or(Path::new("."));

    let file_data = std::fs::read(path).unwrap();
    let elf = elf::ElfBytes::<elf::endian::AnyEndian>::minimal_parse(file_data.as_slice()).unwrap();
    let entry_point = elf.ehdr.e_entry;

    // Create the yaml content using the CONFIG template
    let yaml_content = format!(
        "
set-name: 'PolarFire-SoC-HSS::{elf_name}'

hart-entry-points:
  u54_1: '{entry_point:#x}'
  u54_2: '{entry_point:#x}'
  u54_3: '{entry_point:#x}'
  u54_4: '{entry_point:#x}'

payloads:
  {elf_name}:
    owner-hart: u54_1
    secondary-hart: u54_2
    secondary-hart: u54_3
    secondary-hart: u54_4
    priv-mode: prv_m
    skip-opensbi: true
",
        elf_name = elf_name,
        entry_point = entry_point
    );
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
