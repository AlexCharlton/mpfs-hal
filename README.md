# mpfs-hal

This repository contains a hardware abstraction layer for the Microchip PolarFire SoC, built on the Mirochip-provided [platform](https://github.com/polarfire-soc/platform), as well as [Embassy](https://github.com/embassy-rs/embassy) support and a TTY application which can flash images to a board that is using the [HSS](https://github.com/polarfire-soc/hss) bootloader.

> [!NOTE]
> This repository is an early work in progress. Only the BeagleV-Fire is currently supported, but additional board support should be fairly straightforward.

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

### cargo check

Get cargo to check using the correct target:
```sh
$ cargo check --target riscv64gc-unknown-none-elf
```

## TODO
- Get back to published embassy crates after https://github.com/embassy-rs/embassy/pull/3547 is released
