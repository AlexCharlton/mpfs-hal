# HSS TTY Flasher
This is a TTY interface that allows you to flash ELF files to PolarFire SoC devices using the HSS bootloader. Use it like this:

```sh
hss-tty-flasher <port> <elf>
```

You first need to connect the device to the host via USB as well as a serial cable (like [this one from Adafruit](https://www.adafruit.com/product/954?gQT)). The `port` is the name of the serial port that the device is connected to. On Windows, it will be something like `COM4` (look at the Device Manager to find it), and on Linux and Mac it will be `/dev/tty4` or similar.

The [HSS Payload Generator](https://git.beagleboard.org/beaglev-fire/hart-software-services/-/tree/main-beaglev-fire/tools/hss-payload-generator) is required to generate the image file that is used to flash the ELF file to the device. You can find binaries for it [here](https://github.com/polarfire-soc/hart-software-services/releases).