# mpfs-hal

This repository contains a hardware abstraction layer for the Microchip PolarFire SoC, built on the Mirochip-provided [platform](https://github.com/polarfire-soc/platform), as well as [Embassy](https://github.com/embassy-rs/embassy) support and a TTY application which can flash images to a board that is using the [HSS](https://github.com/polarfire-soc/hss) bootloader.

> [!NOTE]
> This repository is an early work in progress. See crate descriptions below for details about what features are supported. Currently only the BeagleV-Fire board is targeted, but additional board support should be fairly straightforward.

**Primary crates**:

`mpfs-hal` | [![Crates.io](https://img.shields.io/crates/v/mpfs-hal)](https://crates.io/crates/mpfs-hal) [![Docs.rs](https://docs.rs/mpfs-hal/badge.svg)](https://docs.rs/mpfs-hal)<br />
- [critical_section](https://github.com/rust-embedded/critical-section)
- [embedded-alloc](https://github.com/rust-embedded/embedded-alloc) (`alloc` feature)
- Board-specific GPIO ([embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/digital/index.html)) with support for interrupts ([embedded-hal-async](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/digital/index.html))
- UART ([embedded-io](https://docs.rs/embedded-io/latest/embedded_io/)) (Read TODO)
- UART-based logger (`log` and `log-colors` features) and print macros (`print` feature)
- QSPI ([embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiBus.html) and [embedded-hal-async](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/spi/trait.SpiBus.html)) `SpiBus`
- Ethernet and USB support planned next


`mpfs-hal-embassy` | [![Crates.io](https://img.shields.io/crates/v/mpfs-hal-embassy)](https://crates.io/crates/mpfs-hal-embassy) [![Docs.rs](https://docs.rs/mpfs-hal-embassy/badge.svg)](https://docs.rs/mpfs-hal-embassy)<br />
- Embassy integration, with an Executor and Time Driver, supporting multicore with timer interrupts for low-power application.
- Board-specific SD peripheral support via [embassy-embedded-hal](https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/asynch/spi/struct.SpiDevice.html) `SpiDevice` (which implements the [embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiDevice.html) and [embedded-hal-async](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/spi/trait.SpiDevice.html) traits of the same name)


`mpfs-pac` | [![Crates.io](https://img.shields.io/crates/v/mpfs-pac)](https://crates.io/crates/mpfs-pac) [![Docs.rs](https://docs.rs/mpfs-pac/badge.svg)](https://docs.rs/mpfs-pac)<br />
A peripheral access crate for the PolarFire SoC. Largely generated from the [platform](https://github.com/polarfire-soc/platform) repository.


**Utility crates**:

`hss-tty-flasher` | [![Crates.io](https://img.shields.io/crates/v/hss-tty-flasher)](https://crates.io/crates/hss-tty-flasher)<br />
A TTY interface that allows you to flash ELF files to PolarFire SoC devices using the HSS bootloader.


**Internal crates**:

`mpfs-hal-procmacros`<br />
Reexported by `mpfs-hal` and `mpfs-hal-embassy`. Sugar for defining entry points.


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

Additionally, the only flow tested so far uses HSS as the bootloader. Installing the [HSS Payload Generator](https://git.beagleboard.org/beaglev-fire/hart-software-services/-/tree/main-beaglev-fire/tools/hss-payload-generator) is required.

## Usage
See the [examples](https://github.com/AlexCharlton/mpfs-hal/tree/main/examples):
```sh
$ cd examples
$ cargo build --bin embassy-multicore
$ hss-tty-flasher COM5 ../target/riscv64gc-unknown-none-elf/debug/embassy-multicore
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
