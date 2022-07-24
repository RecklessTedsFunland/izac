---
title: Setup Picamera on Ubuntu Server for OpenCV
date: 7 Sept 2020
---

If you don't want to use `picamera` (python only) and want OpenCV (C++ or python) to access
the camera (`/dev/video0`), then do:

- Edit `/boot/firmware/config.txt` and add the following to the bottom:
```
start_x=1
```
- Reboot
- You should see `/dev/video0`


*Note:* for some reason, adding this to `/boot/firmware/usercfg.txt` didn't work.

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
