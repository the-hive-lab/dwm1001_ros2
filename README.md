# DWM1001 ROS2

This repository tracks the documentation and source for the ROS2 driver and support packages related to the [Qorvo DWM1001](https://www.qorvo.com/products/p/DWM1001-DEV) UWB sensor.
The HIVE Lab uses these sensors as a reasonable, low cost alternative to a full motion capture system for locating robots in a space.

## Dependencies

This package requires the installation of our [`pydwm1001`](https://github.com/the-hive-lab/pydwm1001) library, which is not an apt package.
Instead, you can create a virtual environment within your ROS2 workspace and install `pydwm1001` for use by the ROS2 driver:

```
~/ros_ws$ mkdir venv
~/ros_ws$ python3 -m venv --system-site-packages venv/
~/ros_ws$ source venv/bin/activate
(venv) ~/ros_ws$ pip install pydwm1001
```

> The second command assumes you already source the ROS2 environment, e.g. `source /opt/ros/jazzy/setup.bash`, to put ROS2 python libraries on your path.




## Packages

- [`dwm1001_driver`](dwm1001_driver/README.md): The core ROS2 driver.
- [`dwm1001_launch`](dwm1001_launch/README.md): The launch configurations for active and passive node.
- [`dwm1001_transform`](dwm1001_transform/README.md): A package that provides transformations from the `dwm1001` frame to the `map` frame.
- [`dwm1001_visualization`](dwm1001_visualization/README.md): This package provides visualizations in `rviz` for a DWM1001 deployment.