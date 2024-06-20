---
title: Setup Picamera on Ubuntu Server for OpenCV
date: 20 June 2024
---

> **WARNING:**
>
> `picamera` software is not really supported anymore and doesn't support ARM64, only ARM32.
> The underlying library (`mmal`) appears to only be 32bit and the maintainers don't
> have time to invest on 64bit.

## `apt`

I installed a bunch of stuff ... not sure what is needed.

```bash
libcamera-tools/noble,now 0.2.0-3fakesync1build6 arm64 [installed]
libcamera-v4l2/noble,now 0.2.0-3fakesync1build6 arm64 [installed]
libcamera0.2/noble,now 0.2.0-3fakesync1build6 arm64 [installed,automatic]
ros-jazzy-libcamera/noble,now 0.3.0-3noble.20240528.134759 arm64 [installed]

libv4l2rds0t64/noble,now 1.26.1-4build3 arm64 [installed,automatic]
```

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

```bash
$ v4l2-ctl --list-devices
bcm2835-codec-decode (platform:bcm2835-codec):
	/dev/video10
	/dev/video11
	/dev/video12
	/dev/video18
	/dev/video31
	/dev/media3

bcm2835-isp (platform:bcm2835-isp):
	/dev/video13
	/dev/video14
	/dev/video15
	/dev/video16
	/dev/video20
	/dev/video21
	/dev/video22
	/dev/video23
	/dev/media0
	/dev/media1

mmal service 16.1 (platform:bcm2835_v4l2-0):
	/dev/video0

rpivid (platform:rpivid):
	/dev/video19
	/dev/media2


```

```bash
$ v4l2-ctl -d /dev/video0 --list-ctrls

User Controls

                     brightness 0x00980900 (int)    : min=0 max=100 step=1 default=50 value=50 flags=slider
                       contrast 0x00980901 (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                     saturation 0x00980902 (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                    red_balance 0x0098090e (int)    : min=1 max=7999 step=1 default=1000 value=1000 flags=slider
                   blue_balance 0x0098090f (int)    : min=1 max=7999 step=1 default=1000 value=1000 flags=slider
                horizontal_flip 0x00980914 (bool)   : default=0 value=0
                  vertical_flip 0x00980915 (bool)   : default=0 value=0
           power_line_frequency 0x00980918 (menu)   : min=0 max=3 default=1 value=1 (50 Hz)
                      sharpness 0x0098091b (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                  color_effects 0x0098091f (menu)   : min=0 max=15 default=0 value=0 (None)
                         rotate 0x00980922 (int)    : min=0 max=360 step=90 default=0 value=0 flags=modify-layout
             color_effects_cbcr 0x0098092a (int)    : min=0 max=65535 step=1 default=32896 value=32896

Codec Controls

             video_bitrate_mode 0x009909ce (menu)   : min=0 max=1 default=0 value=0 (Variable Bitrate) flags=update
                  video_bitrate 0x009909cf (int)    : min=25000 max=25000000 step=25000 default=10000000 value=10000000
         repeat_sequence_header 0x009909e2 (bool)   : default=0 value=0
                force_key_frame 0x009909e5 (button) : value=0 flags=write-only, execute-on-write
          h264_minimum_qp_value 0x00990a61 (int)    : min=0 max=51 step=1 default=0 value=0
          h264_maximum_qp_value 0x00990a62 (int)    : min=0 max=51 step=1 default=0 value=0
            h264_i_frame_period 0x00990a66 (int)    : min=0 max=2147483647 step=1 default=60 value=60
                     h264_level 0x00990a67 (menu)   : min=0 max=13 default=11 value=11 (4)
                   h264_profile 0x00990a6b (menu)   : min=0 max=4 default=4 value=4 (High)

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=0 value=0 (Auto Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=10000 step=1 default=1000 value=1000
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0
             auto_exposure_bias 0x009a0913 (intmenu): min=0 max=24 default=12 value=12 (0 0x0)
      white_balance_auto_preset 0x009a0914 (menu)   : min=0 max=10 default=1 value=1 (Auto)
            image_stabilization 0x009a0916 (bool)   : default=0 value=0
                iso_sensitivity 0x009a0917 (intmenu): min=0 max=4 default=0 value=0 (0 0x0)
           iso_sensitivity_auto 0x009a0918 (menu)   : min=0 max=1 default=1 value=1 (Auto)
         exposure_metering_mode 0x009a0919 (menu)   : min=0 max=3 default=0 value=0 (Average)
                     scene_mode 0x009a091a (menu)   : min=0 max=13 default=0 value=0 (None)

JPEG Compression Controls

            compression_quality 0x009d0903 (int)    : min=1 max=100 step=1 default=30 value=30

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
import time

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
    time.sleep(0.1)

# When everything done, release the capture
cap.release()
cv2.imwrite("image.png", gray)

#cv2.destroyAllWindows()
```

# References

- Ubuntu: [How to use the Raspberry Pi High Quality camera on Ubuntu Core](https://ubuntu.com/blog/how-to-stream-video-with-raspberry-pi-hq-camera-on-ubuntu-core)
- [Setup Camera and Video Encoder Performance](https://www.codeinsideout.com/blog/pi/set-up-camera/#test-camera)
- [`config.txt`](https://www.raspberrypi.com/documentation/computers/config_txt.html#memory-options)
