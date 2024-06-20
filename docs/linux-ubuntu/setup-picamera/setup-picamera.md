---
title: Setup Picamera on Ubuntu Server for OpenCV
date: 20 June 2024
---

> **WARNING:**
>
> `picamera` software is not really supported anymore and doesn't support ARM64, only ARM32.
> The underlying library (`mmal`) appears to only be 32bit and the maintainers don't
> have time to invest on 64bit.

## Set `/boot/firmware/config.txt`

Add `start_x=1`,`gpu_mem=256` and `dtoverlay=imx219,cam0` to `config.txt`. The
full output is:

```bash
$ cat /boot/firmware/config.txt 
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
#camera_auto_detect=1 << comment out
dtoverlay=imx219,cam0
display_auto_detect=1

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
gpu_mem=256 # on Pi4, should be able to do 256, 128 otherwise
```

## Is the Camera There?

| `vcgencmd get_camera`    | Results  |
|--------------------------|----------|
| `supported=1 detected=1` | All good |
| `supported=0 (or -1)`    | Need to enable camera support in `config.txt` or use `raspi-config` |
| `supported=1 detected=0` | Electrical connection error or bad camera |

```bash
$ vcgencmd get_camerara
supported=1 detected=1, libcamera interfaces=0
```

```bash
$ i2cdetect -y 0
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: 10 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- 64 -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --      
```

The camerea i2c controls are on `/dev/i2c-0`, so I assume what is found is part of the camera.

```bash
v4l2-ctl --list-formats-ext  # list supported formats
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all
v4l2-ctl --list-formats
```

## Add User to Video Group

```bash
sudo usermod -aG video <user_name>
```

## Simple Python OpenCV Test

```python
#!/usr/bin/env python3

import numpy as np
import cv2

cap = cv2.VideoCapture(0)
gray = None

#while(True):
for _ in range(5):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    #cv2.imshow('frame',gray)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

# When everything done, release the capture
cap.release()
cv2.imwrite("image.png", gray)

#cv2.destroyAllWindows()
```

# References

- Ubuntu: [How to use the Raspberry Pi High Quality camera on Ubuntu Core](https://ubuntu.com/blog/how-to-stream-video-with-raspberry-pi-hq-camera-on-ubuntu-core)
- [Setup Camera and Video Encoder Performance](https://www.codeinsideout.com/blog/pi/set-up-camera/#test-camera)
- [`config.txt`](https://www.raspberrypi.com/documentation/computers/config_txt.html#memory-options)
