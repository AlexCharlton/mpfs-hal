# mpfs-pac

This is a Peripheral Access Crate (PAC) for the Microchip PolarFire SoC. It is build on the Mirochip-provided [platform](https://github.com/polarfire-soc/platform) and bindgen.

Because of this approach, this is a fairly gnarly PAC, with no structure to speak of. On the flip side, there's a lot of advanced functionality available. See https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples for usage examples for the platform.

Currently only the BeagleV-Fire board is supported, but adding new boards is a matter of adding the board-specific files to the `mpfs-platform/boards` directory and enabling it through a feature flag in `build.rs`.

## Usage
You should almost certainly use this with the [mpfs-hal](https://github.com/mpfs-hal/mpfs-hal) crate, which - if nothing else - provides macros for entrypoints. If you don't want to use the HAL, you _must_ define of the following entrypoints yourself (one for each hart):
- `u54_1`
- `u54_2`
- `u54_3`
- `u54_4`


## TODO
There's plenty of functionality in platform that isn't yet exposed, either because they are static functions, or defines that aren't recognized by bindgen. I also haven't added all headers to the wrapper, but most MSS peripherals are there.

The docs for this crate are a bit broken, presumably due to some bindgen issue.
