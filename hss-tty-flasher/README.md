# HSS TTY Flasher
This is a TTY interface that allows you to program ELF files to PolarFire SoC devices using the HSS bootloader. Use it like this:

```sh
hss-tty-flasher <port> <elf>
```

You first need to connect the device to the host via USB as well as a serial cable (like [this one from Adafruit](https://www.adafruit.com/product/954?gQT)). The `port` is the name of the serial port that the device is connected to. On Windows, it will be something like `COM4` (look at the Device Manager to find it), and on Linux and Mac it will be `/dev/tty4` or similar.

The [HSS Payload Generator](https://git.beagleboard.org/beaglev-fire/hart-software-services/-/tree/main-beaglev-fire/tools/hss-payload-generator) is required to generate the image file that is used to program the ELF file to the device. You can find binaries for it [here](https://github.com/polarfire-soc/hart-software-services/releases).

## Programming over UART
By default, this program uses the HSS's `USBDMSC` command to expose the eMMC as a USB mass storage device. In situations where the USB port is not accessible, it may be desirable to program the PolarFire over the serial line.

In this case, you can pass the argument `--method uart` to set the programming method. This uses the HHS's `YMODEM` command, so you'll need to have a HSS build that has this enabled. The [bvf-hss-base](https://github.com/AlexCharlton/bvf-gateware-hss-base) project has this set in its default configuration (but the BVF does not). It's also recommended to increase the baud rate used by HSS. This can be done in the HSS source directory with `make config` (General Configuration Options -> Miscellaneous -> Serial Port -> UART baud rate), or simply editing the value of `CONFIG_UART_BAUD_RATE` in `.config`. After doing so, you can set the corresponding rate of hss-tty-flasher with the `--baud` argument. Remember to update your application's UART baud rate as well. With the above mentioned serial cable, a max rate of 1500000 baud has been tested. The [documented max](https://ww1.microchip.com/downloads/aemDocuments/documents/FPGA/ProductDocuments/DataSheets/PolarFire-SoC-Datasheet-DS00004248.pdf) supported by the MPFS is 6.25 Mbps.