# I2C

Enable in firmware:

```ini
$ sudo cat /boot/firmware/config.txt 
[all]
kernel=vmlinuz
cmdline=cmdline.txt
initramfs initrd.img followkernel

[pi4]
max_framebuffers=2
arm_boost=1

[all]
# Enable the audio output, I2C and SPI interfaces on the GPIO header. As these
# parameters related to the base device-tree they must appear *before* any
# other dtoverlay= specification
dtparam=audio=on
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
dtparam=spi=on

# Comment out the following line if the edges of the desktop appear outside
# the edges of your display
disable_overscan=1

# If you have issues with audio, you may try uncommenting the following line
# which forces the HDMI output into HDMI mode instead of DVI (which doesn't
# support audio output)
#hdmi_drive=2

# Enable the serial pins
enable_uart=1

# Autoload overlays for any recognized cameras or displays that are attached
# to the CSI/DSI ports. Please note this is for libcamera support, *not* for
# the legacy camera stack
camera_auto_detect=0
#dtoverlay=imx219,cam0
#display_auto_detect=1

# Config settings specific to arm64
arm_64bit=1
dtoverlay=dwc2

# Enable the KMS ("full" KMS) graphics overlay, leaving GPU memory as the
# default (the kernel is in control of graphics memory with full KMS)
dtoverlay=vc4-kms-v3d
disable_fw_kms_setup=1

[pi3+]
# Use a smaller contiguous memory area, specifically on the 3A+ to avoid an
# OOM oops on boot. The 3B+ is also affected by this section, but it shouldn't
# cause any issues on that board
dtoverlay=vc4-kms-v3d,cma-128

[pi02]
# The Zero 2W is another 512MB board which is occasionally affected by the same
# OOM oops on boot.
dtoverlay=vc4-kms-v3d,cma-128

[all]

[cm4]
# Enable the USB2 outputs on the IO board (assuming your CM4 is plugged into
# such a board)
dtoverlay=dwc2,dr_mode=host

[all]
start_x=1
gpu_mem=256
```

## Check

```bash
#!/bin/bash
# Print current maximum i2c rate
var="$(xxd /sys/class/i2c-adapter/i2c-1/of_node/clock-frequency | awk -F': ' '{print $2}')"
var=${var//[[:blank:].\}]/}
printf "I2C Clock Rate: %d Hz\n" 0x$var

i2cdetect -y 1
```
