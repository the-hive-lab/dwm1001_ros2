# DWM1001 Launch Package

This ROS 2 package contains launch files for the DWM1001 indoor location sensor.

## Launch Files

Currently, we have two lauch files:
+ [`passive_node.launch.py`](launch/passive_node.launch.py) - Fires up a passive node that listens to all location reports and publishes them within ROS2 topics.
+ [`active_node.launch.py`](launch/active_node.launch.py) - Starts an active node that would normally be used with a robot, or other ROS2 enabled device. It publishes the location data for physically attached DWM1001 sensor.

## Parameter Files

TBD
