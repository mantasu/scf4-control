# ROS Control Package for SCF4-M Module
## Overview

This package provides publisher-subscriber scripts for [**SCF4-M**](https://wiki.kurokesu.com/books/scf4) motor controller (targeted at [**C1 PRO**](https://wiki.kurokesu.com/books/c1-pro-x18) camera) for control interaction via [ROS](https://www.ros.org/) messages. The camera has `2` stepper motors [_focus_ | _zoom_] to drive the **FIZ+** lens.

## Temporary section
Rough TODO:
* Extend `SerialHandler` and overwrite some general methods to specialized ones for efficiency
* Autofocus
* Zoom macros (e.g., `zoom 70%`)
* ROS messages for compressed video
* Instructions for `catkin_make`, `chmod +x`
* Dependencies in xml

Currently working on: structure, naming, functionality
Version: 1.1.0

## Running the code

### Requirements
The code was implemented using _Python 3.8.10_ and the libraries listed in [requirements.txt](requirements.txt). You can install these libraries using `conda` (see [this](https://stackoverflow.com/questions/51042589/conda-version-pip-install-r-requirements-txt-target-lib)) or via `pip` by running the following command:

```shell
$ pip install --upgrade pip
$ pip install -r requirements.txt
```

The package was created using _ROS Noetic_. To control the camera lens with keyboard, please install the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package:

```shell
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
```

For image messages, [cv_bridge](http://wiki.ros.org/cv_bridge) is required, please install the package:

```shell
$ sudo apt-get install ros-noetic-cv-bridge
```

> For _Windows_, if there are any issues with detecting the proper USB port (STM32 Virtual COM), install the driver from [STMicroelectronics](https://www.st.com/en/development-tools/stsw-stm32102.html).

### Controlling the Lens

After the camera is connected to the port and `roscore` is running, just run the following to control the lens motors with ROS [geometry_msgs.msg.Twist](http://wiki.ros.org/geometry_msgs) messages via keyboard:

```shell
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

To publish the control command to change lens motor properties, just run:
```
$ TODO
```

Visualize the **C1 PRO** camera output by running:

```
$ TODO
```

### Publishing messages

Start recording

```shell
$ rostopic pub -1 /cam_in scf4_control/CamControl '{start_recording: true, record_duration: 10}'
```


## Configuration file `config.json`
### Topics
* `motors_sub` - the name of the topic to subscribe to where [Scf4Control](msg/Scf4Control.msg) messages are published to control the motors via **SCF4** controller (e.g., to zoom in/out). This is used by _c1prox18_ node.
* `camera_sub` - the name of the topic to subscribe to where [CamControl](msg/CamControl.msg) messages are published to control the streamer (e.g., to record). This is used by _c1prox18_ node.
* `camera_pub` - the name of the topic to which [CompressedImage](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/CompressedImage.html) messages are published (i.e., camera frames). Messages are published by _c1prox18_ node which should work at 30 Hz because the camera's max frame rate is 30. The _viewer_ node subscribes to this topic to show what the camera captures on a separate window.
### Capturer
* `device_id` - the ID of the capture device (camera). If **C1 Pro** is the only camera, use `0`.
* `fps` - the framerate at which to capture camera frames. This should not be bigger than `30` which is camera's max FPS as specified in the official site.
* `width` - the resolution width. The smaller the width, the faster the frames are captured and the faster they are processed (e.g., for focusing). Larger values increase the video quality but may cause delays if frames need to be processed fast and may lead to failure. Note that the width should be compatible with the height and available resolutions can be checked below in section **C1 Pro X18 Properties**.
* `height` - the resolution height. The smaller the height, the faster the frames are captured and the faster they are processed (e.g., for focusing). Larger values increase the video quality but may cause delays if frames need to be processed fast and may lead to failure. Note that the height should be compatible with the width and available resolutions can be checked below in section **C1 Pro X18 Properties**.
* `fourcc` - 4-character code of codec used to compress the frames. List of codes can be obtained at [Video Codecs by FOURCC](https://www.fourcc.org/codecs/) page. The ones supported by the camera are `MJPG` (runs at 30 FPS), `H264` (runs at 30 FPS) and `YUYV` (runs at 5 (when _1920x1080_), 10 (when _1280×720_), 30 (when _640×480_) FPS).
* `backend` - enumerator for video capture API to use as defined in [Flags for video I/O](https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html). By default, it is `-1` in which case **V4L2** is chosen for _Linux_, **AVFoundation** for _Mac_, **DirectShow** for _Windows_, or _auto_ for other OS.
* `format` - the format in which the captured images/frames should be converted to.
* `delay` - the delay between a real-time frame and a captured frame that's been read into memory.

### Recorder
* `fps` - the framerate at which to record camera frames. This should not be bigger than capturer's FPS.
* `width` - same as capturer's width. It can be smaller though than the capturer's width in case the recording size needs to be of smaller size.
* `height` - same as capturer's height. It can be smaller though than the capturer's height in case the recording size needs to be of smaller size.
* `fourcc` - same as capturer's, except it is not restricted to camera's properties so it can be an arbitrary choice. The default is `avc1` as it supports `mp4` format and works efficiently. 
* `format` - the format in which the video file should be saved. Assure it is supported by FOURCC
* `out_dir` - the directory to save the captured video files by the camera. If the directory(-ies) does not exist, it is created automatically. Just make sure the program has proper permissions to create directories/files.
* `is_relative` - whether the `out_dir` path is relative to the package directory. If `false`, an absolute directory of the whole system is used.

### Serial
* `port` - the USB port to which the **SCF4** controller is connected, i.e., the path to the device.
* `baudrate` - the rate (max bits per second) at which the information is transferred in a communication between a serial port and the **SCF4** controller. The manufacturer says it shouldn't matter.
* `timeout` - the time (in seconds) to wait before terminating the waiting process for the response after sending a command through serial.
### Motors
* `min_idle_time` - the minimum time (in seconds) to wait to consider the motor is not moving. This is, for example, used to start the focusing process after the zoom motor is stopped.
* `steps_def` - the default number of jog steps to make after a move forward/backward command is issued via serial
* `steps_min` - the minimum number of steps the motor can make to move forward/back in one go, any value lower than that is clamped to it
* `steps_max` - the maximum number of steps the motor can make to move forward/back in one go, any value higher than that is clamped to it
* `speed_def` - the default (e.g., starting) speed at which the motor should move. Note that speed is a 16-bit register which specifies internal timing interval, thus the lower the value, the faster the pulse rate is.
* `speed_min` - the minimum speed value the motor can have, any value lower than that is clamped to it. Again, note that the lower the value the faster the motor moves, thus this value specifies the fastest motor speed.
* `speed_max` - the maximum speed value the motor can have, any value higher than that is clamped to it. Again, note that the higher the value the slower the motor moves, thus this value specifies the slowest motor speed.
* `count_def` - the internal position counter at which the motor should be in a default state (e.g., when the controller boots up). Note that the position counter value is a 16-bit unsigned integer, however, the motor does not function within such large range (i.e., `0` to `65535`), it functions within some fraction of that range and the fraction depends on the motor type.
* `count_min` - the minimum value of the internal counter which should correspond to the mechanical minimum of the motor.
* `count_max` - the maximum value of the internal counter which should correspond to the mechanical maximum of the motor.
* `switch_pos` - the position (in terms of the motor internal counter) of the switch trigger/sensor which is located approximately in the middle of the mechanical motor's moving range. It should be used to configure the counter values to correspond to the mechanical positions, i.e., it should be used to set the middle of the position counter range that is specified by `count_min` and `count_max`.
* `vel_factor` - the velocity factor which is multiplied by the linear `x` component (if the motor is _zoom_) or by the angular `z` component (if the motor is _focus_) of the [Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) message sent by [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package in case the camera is controlled using that package. This factor converts those components o realistic values that can be interpreted by the **SCF4** controller

## C1 Pro X18 Properties

The camera has been tested for a range of resolutions and the following passed the tests:

| Width  |     | Height | Result |
| -----: | :-: | :----- | :----: |
| `640`  | x   | `480`  | ✔      |
| `720`  | x   | `480`  | ✔      |
| `720`  | x   | `576`  | ✔      |
| `800`  | x   | `600`  | ✔      |
| `1280` | x   | `720`  | ✔      |
| `1280` | x   | `1024` | ✔      |
| `1920` | x   | `1080` | ✔      |
