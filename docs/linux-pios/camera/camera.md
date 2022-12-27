# Camera Setup CM4 Stereo Cameras

Raspberry Pi OS 64-bit

Change `/boot/config.txt` to the following and reboot.

```
# Automatically load overlays for detected cameras
#camera_auto_detect=1
dtoverlay=imx219,cam1
dtoverlay=imx219,cam0
```

```
pi@cm4:~ $ libcamera-jpeg --list-cameras
Available cameras
-----------------
0 : imx219 [3280x2464] (/base/soc/i2c0mux/i2c@0/imx219@10)
    Modes: 'SRGGB10_CSI2P' : 640x480 [206.65 fps - (1000, 752)/1280x960 crop]
                             1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
                             1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
                             3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
           'SRGGB8' : 640x480 [206.65 fps - (1000, 752)/1280x960 crop]
                      1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
                      1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
                      3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
1 : imx219 [3280x2464] (/base/soc/i2c0mux/i2c@1/imx219@10)
    Modes: 'SRGGB10_CSI2P' : 640x480 [206.65 fps - (1000, 752)/1280x960 crop]
                             1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
                             1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
                             3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
           'SRGGB8' : 640x480 [206.65 fps - (1000, 752)/1280x960 crop]
                      1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
                      1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
                      3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
```


## References

[Raspberry Pi Docs](https://www.raspberrypi.com/documentation/accessories/camera.html#libcamera-hello)

From the [Waveshare wiki](https://www.waveshare.com/wiki/CM4-IO-BASE-B)

```
#Use dual cameras in the new system
#Remove "camera_auto_detect=1" in config.txt
#camera_auto_detect=1

#Add
dtoverlay=imx219,cam1
dtoverlay=imx219,cam0

#imx219 is the model of the camera sensor, and it also supports other sensors.
dtoverlay=ov5647,cam0
dtoverlay=imx219,cam0
dtoverlay=ov9281,cam0
dtoverlay=imx477,cam0
dtoverlay=imx519,cam0

#Then reboot
reboot

#Open the camera
libcamera-hello -t 0
or
libcamera-hello

#Other commands:
#Check whether the camera is detected
libcamera-hello --list-cameras

#Open the corresponding camera for previewing 5 seconds
libcamera-hello  --camera 1
libcamera-hello  --camera 0

#Take a picture
libcamera-jpeg -o test.jpg

#Record vedio
libcamera-vid -t 10000 -o test.h264

#You can add "--camera" to specify the camera.
#-t <duration> allows the user to choose how long the window will be displayed in milliseconds.
```