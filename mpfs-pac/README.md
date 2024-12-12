# MPFS PAC

This is a Peripheral Access Crate (PAC) for the Microchip PolarFire SoC. It is build on the Mirochip-provided [platform](https://github.com/polarfire-soc/platform) and bindgen.

Because of this approach, this is a fairly gnarly PAC. See https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples for usage examples for the platform.

Currently only the BeagleV-Fire board is supported, but adding new boards is a matter of adding the board-specific files to the `mpfs-platform/boards` directory and enabling it through a feature flag in `build.rs`.

## TODO
There's lots of functionality in platform that isn't yet exposed, either because they are static functions, or defines that aren't recognized by bindgen. I also haven't added all headers to the wrapper, since I've been adding them as I need them.

The docs for this crate are a broken, presumably due to some bindgen issue.
