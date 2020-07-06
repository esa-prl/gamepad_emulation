# Gamepad Emulation 

## Overview

This package emulates the input of a gamepad with the computer keyboard. 

**Keywords:** keyboard, gamepad, emulation, package

### License

The source code is released under a [TODO: Add Licence]()).

**Author: Maximilian Ehrhardt<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: maximilian Ehrhardt, maximilian.ehrhardt@esa.int**

The Gamepad Emulation package has been tested under [ROS2] Eloquent and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [pynput](https://pypi.org/project/pynput/)

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/gamepad_emulations.git
	cd ../
	colcon build --symlink-install

To install the dependency pynput use
	rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent 

Adding `--symlink-install` to `colcon build` eliminates the need to recompile the package after changing the code.


## Usage

Run the main node with:

	ros2 run gamepad_emulation gamepad_emulation_node

## Nodes

### gamepad_emulation_node

Emulates the input of a gamepad by using the computer keyboard.

#### Published Topics

* **`/joy`** ([/sensor_msgs/Joy])

    Containing emulated buttons pressed and emulated movement of joystick. 

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rover_msgs/ChangeLocomotionMode]: https://github.com/esa-prl/rover_msgs/blob/master/srv/ChangeLocomotionMode.srv
[sensor_msgs/Joy]: http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html
[geometry_msgs/Twist]: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html