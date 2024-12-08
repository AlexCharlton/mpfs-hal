# mpfs-hal

This repository contains a hardware abstraction layer for the Microchip PolarFire SoC, built on the Mirochip-provided [platform](https://github.com/polarfire-soc/platform), as well as [Embassy](https://github.com/embassy-rs/embassy) support and a TTY application which can flash images to a board that is using the [HSS](https://github.com/polarfire-soc/hss) bootloader.

> [!NOTE]
> This repository is an early work in progress. There is not much HAL to speak of. Only the BeagleV-Fire is currently supported, but additional board support should be fairly straightforward.

**Primary crates**:
- `mpfs-hal`: [critical_section](https://github.com/rust-embedded/critical-section), [embedded-alloc](https://github.com/rust-embedded/embedded-alloc), integration. [embedded-hal](https://github.com/rust-embedded/embedded-hal) support intended.
- `mpfs-hal-embassy`: Embassy integration, with an Executor and Time Driver, supporting multicore with timer interrupts for low-power application.
- `mpfs-pac`: A peripheral access crate for the PolarFire SoC.

**Utility crates**:
- `hss-tty-flasher`: A TTY interface that allows you to flash ELF files to PolarFire SoC devices using the HSS bootloader.

**Internal crates**:
- `mpfs-hal-procmacros`: Reexported by `mpfs-hal` and `mpfs-hal-embassy`. Sugar for defining entry points.


## Prerequisites
When checking out this repository, clone the submodules as well:
```sh
$ git clone --recursive
```

You will need a RISCV toolchain. The [SoftConsole](https://www.microchip.com/en-us/products/fpgas-and-plds/fpga-and-soc-design-tools/soc-fpga/softconsole)-supplied one will work.

The RISC-V target can be added using rustup:
```sh
$ rustup target add riscv64gc-unknown-none-elf
```

Nightly Rust is used for the embassy examples.

```sh
$ rustup install nightly
```

Additionally, the only flow tested so far uses HSS as the bootloader. Installing the [HSS Payload Generator](https://git.beagleboard.org/beaglev-fire/hart-software-services/-/tree/main-beaglev-fire/tools/hss-payload-generator) is required.

## Usage
See the examples:
```sh
$ cd examples
$ cargo build --bin embassy-multicore
$ hss-tty-flasher COM5 target/riscv64gc-unknown-none-elf/debug/embassy-multicore
```

Your application's `.cargo/config.toml` should specify both the target and the linker file:
```toml
[build]
target = "riscv64gc-unknown-none-elf"

[target.riscv64gc-unknown-none-elf]
rustflags = ["-C", "link-arg=-Tlinker.ld", "-C", "link-arg=--gc-sections"]
```

This ensures that the [linker script](https://github.com/AlexCharlton/mpfs-hal/blob/main/mpfs-pac/linker.ld) is applied.


### cargo check

Get cargo to check using the correct target:
```sh
$ cargo check --target riscv64gc-unknown-none-elf
```

## TODO
- Get back to published embassy crates after https://github.com/embassy-rs/embassy/pull/3547 is released
