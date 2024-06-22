# Pi Camera v2.1

- `image_tools`: `ros2 run image_tools cam2image` reads `/dev/video0` and publishes on `/image`
  - `ros2 run image_tools cam2image --ros-args -p width:=1920 -p height:=1080`
- `v4l2_camera`: `ros2 run v4l2_camera v4l2_camera_node`
  - `apt install ros-${ROS_DISTRO}-v4l2-camera`
  - `apt install ros-${ROS_DISTRO}-image-transport-plugins`
  - `ros2 param set /v4l2_camera image_size [1280,720]`
  - docs: https://gitlab.com/boldhearts/ros2_v4l2_camera/-/tree/jazzy?ref_type=heads
- [ROS2 libcamera node](https://github.com/christianrauch/camera_ros): `sudo apt install ros-$ROS_DISTRO-camera-ros`
  - **WARNING:** doesn't seem to be working for jazzy right now (2024-06-20)

## `v4l2_camera_node`

```
$ ros2 topic list
/camera_info
/image_raw
/image_raw/compressed
/image_raw/compressedDepth
/image_raw/theora
/image_raw/zstd
/parameter_events
/rosout
```

## V4L2

`camera_autodetect=0`

```bash
$ v4l2-ctl -D
Driver Info:
	Driver name      : unicam
	Card type        : unicam
	Bus info         : platform:fe801000.csi
	Driver version   : 6.8.4
	Capabilities     : 0xa5a00001
		Video Capture
		Metadata Capture
		I/O MC
		Read/Write
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x25200001
		Video Capture
		I/O MC
		Read/Write
		Streaming
		Extended Pix Format
Media Driver Info:
	Driver name      : unicam
	Model            : unicam
	Serial           : 
	Bus info         : platform:fe801000.csi
	Media version    : 6.8.4
	Hardware revision: 0x00000000 (0)
	Driver version   : 6.8.4
Interface Info:
	ID               : 0x03000005
	Type             : V4L Video
Entity Info:
	ID               : 0x00000003 (3)
	Name             : unicam-image
	Function         : V4L2 I/O
	Flags            : default
	Pad 0x01000004   : 0: Sink
	  Link 0x02000007: from remote pad 0x1000002 of entity 'imx219 10-0010' (Camera Sensor): Data, Enabled, Immutable
```

`dtoverlay=imx219,cam0`

```bash
$ v4l2-ctl -D
Driver Info:
	Driver name      : bcm2835 mmal
	Card type        : mmal service 16.1
	Bus info         : platform:bcm2835_v4l2-0
	Driver version   : 6.8.4
	Capabilities     : 0x85200005
		Video Capture
		Video Overlay
		Read/Write
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x05200005
		Video Capture
		Video Overlay
		Read/Write
		Streaming
		Extended Pix Format
```

## Debug

```bash
ros2 run rqt_image_view rqt_image_view
```

## Compression

- [zstd](http://facebook.github.io/zstd/)
