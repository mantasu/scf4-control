# ROS Control Package for SCF4-M Module
## Overview

This package provides publisher-subscriber scripts for [**SCF4-M**](https://wiki.kurokesu.com/books/scf4) motor controller (targeted at [**C1 PRO**](https://wiki.kurokesu.com/books/c1-pro-x18) camera) for control interaction via [ROS](https://www.ros.org/) messages. The camera has `2` stepper motors [_focus_ | _zoom_] to drive the **FIZ+** lens.

## Temporary section
Rough TODO:
* Extend `SerialHandler` and overwrite some general methods to specialized ones for efficiency
* Autofocus
* Zoom macros (e.g., `zoom 70%`)
* ROS messages for compressed video

Currently working on: structure, naming, functionality
Version: 0.1.0

## Running the code

### Requirements
The code was implemented using _Python 3.8.10_ and the libraries listed in [requirements.txt](requirements.txt). You can install these libraries using `conda` (see [this](https://stackoverflow.com/questions/51042589/conda-version-pip-install-r-requirements-txt-target-lib)) or via `pip` by running the following command:

```shell
$ pip install -r requirements.txt
```

The package was created using _ROS Noetic_. To control the camera lens with keyboard, please install the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package:

```shell
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
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