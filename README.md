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

### Capture
* `fourcc` - 4-character code of codec used to compress the frames. List of codes can be obtained at [Video Codecs by FOURCC](https://www.fourcc.org/codecs/) page. For best quality/compression, please use one of the following: `mp4v` | `mjpg` | `avc1`
* `backend` - enumerator for video capture API to use as defined in [Flags for video I/O](https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html). By default, it is `-1` in which case **V4L2** is chosen for _Linux_, **AVFoundation** for _Mac_, **DirectShow** for _Windows_, or _auto_ for other OS.

### Writer
* `out_dir` - the directory to save the captured video files by the camera. If the directory(-ies) does not exist, it is created automatically. Just make sure the program has proper permissions to create directories/files.





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