use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::PathBuf;
use std::process::Command;

pub fn list_removable_drives() -> Vec<PathBuf> {
    #[cfg(windows)]
    {
        use windows::core::PCWSTR;
        use windows::Win32::Storage::FileSystem::{GetDriveTypeW, GetLogicalDrives};
        use windows::Win32::System::WindowsProgramming::DRIVE_REMOVABLE;

        let mut drives = Vec::new();
        let bitmask = unsafe { GetLogicalDrives() };

        for i in 0..26 {
            if (bitmask & (1 << i)) != 0 {
                let drive_letter = char::from(b'A' + i as u8);
                let path = format!("{}:\\", drive_letter);

                // Convert to wide string for Windows API
                let wide_path: Vec<u16> = path.encode_utf16().chain(std::iter::once(0)).collect();

                // Check if drive is removable
                let drive_type = unsafe { GetDriveTypeW(PCWSTR::from_raw(wide_path.as_ptr())) };
                if drive_type == DRIVE_REMOVABLE {
                    drives.push(PathBuf::from(path));
                }
            }
        }
        drives
    }

    #[cfg(not(windows))]
    {
        use sysinfo::Disks;

        let mut sys = System::new_all();
        sys.refresh_disks_list();

        sys.disks()
            .iter()
            .filter(|disk| disk.is_removable())
            .map(|disk| disk.mount_point().to_string_lossy().into_owned())
            .collect()
    }
}

pub fn eject_drive(mount_point: &PathBuf) -> io::Result<()> {
    #[cfg(target_os = "linux")]
    {
        // On Linux, we can use the 'eject' command
        Command::new("eject").arg(mount_point).output().map(|_| ())
    }

    #[cfg(target_os = "macos")]
    {
        // On macOS, we use 'diskutil eject'
        Command::new("diskutil")
            .args(["eject", mount_point])
            .output()
            .map(|_| ())
    }

    #[cfg(target_os = "windows")]
    {
        // On Windows, we need to use PowerShell to safely eject
        Command::new("powershell")
            .args([
                "-Command",
                &format!(
                    "($driveEject = New-Object -comObject Shell.Application).Namespace(17).ParseName('{}').InvokeVerb('Eject')",
                    mount_point.display()
                ),
            ])
            .output()
            .map(|_| ())
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "Ejecting drives is not supported on this operating system",
        ))
    }
}

pub fn flash_image_to_drive(
    image_path: &str,
    target_drive: &PathBuf,
    log_writer: &mut std::io::BufWriter<File>,
) -> io::Result<()> {
    let mut source = File::open(image_path)?;

    #[cfg(unix)]
    let target_path = format!("/dev/{}", target_drive.display());
    #[cfg(windows)]
    let target_path = format!(
        r"\\.\{}:",
        target_drive
            .to_string_lossy()
            .trim_end_matches('\\')
            .trim_end_matches(':')
    );

    log_writer.write_all(format!("Opening target at: {}\n", target_path).as_bytes())?;
    let mut target = File::options().write(true).open(&target_path)?;
    log_writer.write_all(format!("Opened target at: {}\n", target_path).as_bytes())?;

    // Seek to the beginning of both files
    source.seek(SeekFrom::Start(0))?;
    target.seek(SeekFrom::Start(0))?;

    let mut buffer = vec![0; 512]; // Use 512 bytes (typical sector size)

    loop {
        let bytes_read = source.read(&mut buffer)?;
        if bytes_read == 0 {
            break;
        }
        log_writer.write_all(format!("Read {} bytes\n", bytes_read).as_bytes())?;

        if bytes_read < 512 {
            // Pad the remaining buffer with zeros to complete the sector
            buffer[bytes_read..512].fill(0);
        }
        target.write_all(&buffer[..512])?;
        log_writer.write_all(format!("Wrote {} bytes\n", bytes_read).as_bytes())?;
    }

    Ok(())
}
