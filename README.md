# mpfs-hal

This repository contains a hardware abstraction layer for the Microchip PolarFire SoC, built on the Mirochip-provided [platform](https://github.com/polarfire-soc/platform), as well as [Embassy](https://github.com/embassy-rs/embassy) support and a TTY application which can flash images to a board that is using the [HSS](https://github.com/polarfire-soc/hss) bootloader.

> [!NOTE]
> This repository is a work in progress. See the crate descriptions below for details about what features are supported. Currently only the BeagleV-Fire board is targeted, but additional board support should be fairly straightforward. See https://github.com/AlexCharlton/mpfs-hal/issues/1 for a list of peripherals that do not yet have support.

> [!TIP]
> Using a more recent version of the HSS bootloader is highly recommended. DDR training on earlier versions hangs frequently. See the [**Gateware programming**](#gateware-programming) section for details about upgrading.

**Primary crates**:

`mpfs-hal` | [![Crates.io](https://img.shields.io/crates/v/mpfs-hal)](https://crates.io/crates/mpfs-hal) [![Docs.rs](https://docs.rs/mpfs-hal/badge.svg)](https://docs.rs/mpfs-hal)<br />
- [critical_section](https://github.com/rust-embedded/critical-section)
- [alloc](https://doc.rust-lang.org/alloc/) support via [embedded-alloc](https://github.com/rust-embedded/embedded-alloc) (`alloc` feature)
- Board-specific GPIO ([embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/digital/index.html) `OutputPin` and `InputPin`) with support for interrupts ([embedded-hal-async](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/digital/trait.Wait.html) `Wait`)
- UART ([embedded-io](https://docs.rs/embedded-io/latest/embedded_io/trait.Write.html) `Write` and [embedded-io-async](https://docs.rs/embedded-io-async/latest/embedded_io_async/trait.Read.html) `Read`)
- UART-based logger (`log` and `log-colors` features) and print macros (`print` feature)
- QSPI ([embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiBus.html) and [embedded-hal-async](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/spi/trait.SpiBus.html) `SpiBus`)
- Ethernet ([embassy-net-driver](https://docs.embassy.dev/embassy-net-driver/git/default/index.html) `Driver`)

> [!NOTE]
> While the `mpfs-hal` crate implements some Embassy traits, it comes with no requirement to use Embassy. These traits were used in the absence of other async traits available in the ecosystem, and they come with the benefit of having USB/Ethernet stacks already implemented with [embassy-usb](https://docs.embassy.dev/embassy-usb/git/default/index.html) and [embassy-net](https://docs.embassy.dev/embassy-net/git/default/index.html) (both of which also do not need to be used with the Embassy executor, if so desired).


`mpfs-hal-embassy` | [![Crates.io](https://img.shields.io/crates/v/mpfs-hal-embassy)](https://crates.io/crates/mpfs-hal-embassy) [![Docs.rs](https://docs.rs/mpfs-hal-embassy/badge.svg)](https://docs.rs/mpfs-hal-embassy)<br />
- Embassy integration, with an Executor and Time Driver, supporting multicore with timer interrupts for low-power application.
- Board-specific SD peripheral support via [embassy-embedded-hal](https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/asynch/spi/struct.SpiDevice.html) `SpiDevice` (which implements the [embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiDevice.html) and [embedded-hal-async](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/spi/trait.SpiDevice.html) traits of the same name)
- USB device ([embassy-usb-driver](https://docs.embassy.dev/embassy-usb-driver/git/default/index.html) `Driver`)
- USB host support (using the yet-to-be-released [UsbHostDriver](https://github.com/embassy-rs/embassy/pull/3307)) in the works


`mpfs-pac` | [![Crates.io](https://img.shields.io/crates/v/mpfs-pac)](https://crates.io/crates/mpfs-pac) [![Docs.rs](https://docs.rs/mpfs-pac/badge.svg)](https://docs.rs/mpfs-pac)<br />
A peripheral access crate for the PolarFire SoC. Largely bindgen-generated from the [platform](https://github.com/polarfire-soc/platform) repository.


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

## Gateware programming
See [beaglev-fire-zephyr-and-baremetal-with-gateware](https://github.com/AlexCharlton/beaglev-fire-zephyr-and-baremetal-with-gateware) for an example project that contains the BeagleV-Fire gateware along with an updated bootloader (HSS). [icicle-kit-minimal-bring-up-design-bitstream-builder](https://github.com/polarfire-soc/icicle-kit-minimal-bring-up-design-bitstream-builder) similarly illustrates this for the Icicle Kit (though the HSS it uses is older).

You can find a [pre-compiled bitstream](https://github.com/AlexCharlton/beaglev-fire-zephyr-and-baremetal-with-gateware/releases/tag/default-bitstream-1.0) in the former repo which can either be programmed by using a [FlashPro Express 5 or 6](https://www.microchip.com/en-us/products/fpgas-and-plds/fpga-and-soc-design-tools/programming-and-debug/flashpro-express) or by taking advantage of the MPFS's auto-update feature via the SPI flash memory, like the BeagleV-Fire [`change-gateware.sh` script](https://docs.beagle.cc/boards/beaglev/fire/demos-and-tutorials/gateware/customize-cape-gateware-verilog.html#program-beaglev-fire-with-your-custom-bitstream) does.

If using a FlashPro Express, make sure that the SPI flash does not point to an auto-updateable bitstream. To this end, you can use [this image](https://github.com/AlexCharlton/beaglev-fire-zephyr-and-baremetal-with-gateware/releases/tag/spi-erase-1.0) to erase the the first block of the SPI flash memory. Just program it to the board using hss-tty-flasher and let it boot. This only needs to be done once.
