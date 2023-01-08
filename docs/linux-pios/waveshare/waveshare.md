![](cm4-io-base-a-3.jpg)

# Waveshare CM4-IO-Base-B

**Having problems with this board, won't reliably boot**

- [Waveshare wiki](https://www.waveshare.com/wiki/CM4-IO-BASE-B)
- [Booting CM4 from NVME](https://blog.j2i.net/2022/04/12/booting-a-pi-cm4-on-nvme/)
- [Bootloader Updating on RPi](https://pimylifeup.com/raspberry-pi-bootloader/)

> **WARNING:** You can **NOT** us the SD card if you CM4 has eMMC since they are 
> hooked up to the same IO lines.

## Loading Image on CM4

Follow Jeff Geerling's [video](https://www.youtube.com/watch?v=jp_mF1RknU4) on how to do this.

Jeff also explains how the antenna works, you don't need to
plug in the external [ref](https://www.jeffgeerling.com/blog/2022/enable-external-antenna-connector-on-raspberry-pi-compute-module-4)

1. macOS
    - `brew install libusb`
    - `brew install pkg-config`
1. Download rpiboot: https://github.com/raspberrypi/usbboot
    - `make`
    - `./rpiboot`
    ```
    (py)  kevin@Logan usbboot-master % ./rpiboot
    RPIBOOT: build-date Dec 30 2022 version 20221215~105525 
    Waiting for BCM2835/6/7/2711...
    Loading embedded: bootcode4.bin
    Sending bootcode.bin
    Successful read 4 bytes 
    Waiting for BCM2835/6/7/2711...
    Loading embedded: bootcode4.bin
    Second stage boot server
    Cannot open file config.txt
    Cannot open file pieeprom.sig
    Loading embedded: start4.elf
    File read: start4.elf
    Cannot open file fixup4.dat
    Second stage boot server done
    ```
1. Set Boot selection switch to `on`
1. Don't plug anything in, just the CM4 module
1. Plug in the USB-C cable and the pi will mount as a USB drive
    - The power light should be bright RED, if not, something is wrong
    - My CM4 sometimes flashes 8 times on the activity LED, meaning SDRAM failure, but sometimes it is fine
3. Run RPi Imager, use PiOS Lite, 64-bit

## Update eeprom

> **WARNING:** `sudo rpi-eeprom-update` appears to be disabled for some reason on the 64b version of rpiOS

> **WARNING:** You don't seem to be able to update the firmware on the 64b rpiOS ... wtf?

Mine is old, 2021, while the newest is [2022-12-07](https://github.com/raspberrypi/rpi-eeprom/releases) available from the raspberry pi rpi-eeprom github release page.

See current version of firmware: `vcgencmd bootloader_version`

```
kevin@cm4:~ $ vcgencmd bootloader_version
Feb 16 2021 13:23:36
version d6d82cf99bcb3e9a166a34cfde53130957a36bd3 (release)
timestamp 1613481816
update-time 1613481816
capabilities 0x0000001f
```

Having issues upgrading, since again, can't boot.

Maybe working??????

1. Mount CM4 with `./rpiboot` as shown above
2. Open `Raspberry Pi Imager`
    - OS -> Misc utiliy -> Bootloader -> SD Card Boot
    - Choose drive
    - Write

