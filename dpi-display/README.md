# DPI Display

This is an FPGA-driven DPI display driver.

It's been tested with a [Waveshare 800x480 DPI display](https://www.waveshare.com/5inch-LCD-for-Pi.htm) on the BeagleV Fire.

## Usage
Compile gateware with https://github.com/AlexCharlton/bvf-gateware-hss-base and load it onto the board.

```sh
$ build-gateware gateware/DPI_DISPLAY --program
```

Modify the parameters found in `gateware/DPI_DISPLAY/HDL/display.v` to match the display you are using (and optionally `gateware/DPI_DISPLAY/ADD_CAPE.tcl` to alter the cape pins).

The gateware requires 4MB of reserved memory for the framebuffer. Add to your `.cargo/config.toml`:

```toml
[env]
MPFS_RESERVED_MEMORY_SIZE = "400000"
```

## Running the testbench
The testbench depends on Icarus Verilog (for running fast tests) and Verilator (for running full tests). Waveforms are loaded into [Surfer](https://surfer-project.org/) for viewing.

```sh
$ ./gateware/test
```

Will run the full testbench. `./gateware/test cape` will check to see if the cape compiles. `./gateware/test fast` will run the testbench without generating a VCD.
