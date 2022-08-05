---
title: Setup Picamera on Ubuntu Server for OpenCV
date: 7 Sept 2020
---


> **WARNING:** 
>
> PiCamera **dones not work on ARM64**

> **WARNING:** 
>
> ROS 2 on ARM32 requires you to compile *all* packages yourself. Only ARM64
> has binary precompiled packages.

> **WARNING:**
>
> `picamera` software is not really supported anymore and doesn't support ARM64, only ARM32.
> The underlying library (`mmal`) appears to only be 32bit and the maintainers don't
> have time to invest on 64bit.

## Is the Camera There?

| `vcgencmd get_camera`    | Results  |
|--------------------------|----------|
| `supported=1 detected=1` | All good |
| `supported=0 (or -1)`    | Need to enable camera support in `config.txt` or use `raspi-config` |
| `supported=1 detected=0` | Electrical connection error or bad camera |

```bash
i2cdetect -y 0
```

The camerea i2c controls are on `/dev/i2c-0`, so this should show the camerea, but I 
don't see anything.

```bash
v4l2-ctl --list-formats-ext  # list supported formats
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all
v4l2-ctl --list-formats
```

## Boot Config

If you don't want to use `picamera` (python only) and want OpenCV (C++ or python) to access
the camera (`/dev/video0`), then do:

- Edit `/boot/firmware/config.txt` and add the following to the bottom:
```
start_x=1
gpu_mem=128 # on Pi4, should be able to do 256
```
- Reboot
- You should see `/dev/video0`

> *Note:* there is already a `camera_auto_detect=1` in the `config.txt` that looks
> for the camera.

> *Note:* for some reason, adding this to `/boot/firmware/usercfg.txt` didn't work.

## Add User to Video Group

```bash
sudo usermod -aG video <user_name>
```

## Simple Python OpenCV Test

```python
import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
```

# References

- Ubuntu: [How to use the Raspberry Pi High Quality camera on Ubuntu Core](https://ubuntu.com/blog/how-to-stream-video-with-raspberry-pi-hq-camera-on-ubuntu-core)
- [Setup Camera and Video Encoder Performance](https://www.codeinsideout.com/blog/pi/set-up-camera/#test-camera)
- [`config.txt`](https://www.raspberrypi.com/documentation/computers/config_txt.html#memory-options)
