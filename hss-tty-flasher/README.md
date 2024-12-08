# HSS TTY Flasher
This is a TTY interface that allows you to flash ELF files to PolarFire SoC devices using the HSS bootloader. Use it like

```sh
hss-tty-flasher <port> <elf>
```

The [HSS Payload Generator](https://git.beagleboard.org/beaglev-fire/hart-software-services/-/tree/main-beaglev-fire/tools/hss-payload-generator) is required to generate the image file that is used to flash the ELF file to the device. You can find binaries for it [here](https://github.com/polarfire-soc/hart-software-services/releases).