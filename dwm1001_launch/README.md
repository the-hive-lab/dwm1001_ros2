# DWM1001 Launch Package

This ROS 2 package contains launch files for the DWM1001 indoor location sensor.

## Launch Files

There are three launch files for the DWM1001:
+ [`active_node.launch.py`](launch/active_node.launch.py) - Starts an active node that would normally be used with a robot, or other ROS2 enabled device. It publishes the location data for physically attached DWM1001 sensor.
+ [`passive_node.launch.py`](launch/passive_node.launch.py) - Fires up a passive node that listens to all location reports and publishes them within ROS2 topics.
+ [`transform_node.launch.py`](launch/transform_node.launch.py) - Starts the node that performs transforms from the `dwm1001` frame to the `map` frame. You will need this node if you plan to use something like `robot_localization`.


### Active Node Launch

Launch parameters:
+ `tag_namespace`: If you want to change the namespace, set this parameter to what you want but make sure your config yaml file matches! See the [default example](config/default_active.yaml) to see how to structure your configuration file. Default value is `active`. 
+ `config_file`: The name of the config file you want to use. It must be in the ROS2 workspace under this package's `config` folder. Default value is `default_active.yaml`. 

### Passive Node Launch

Launch parameters:
+ `tag_namespace`: If you want to change the namespace, set this parameter to what you want but make sure your config yaml file matches! See the [default example](config/default_passive.yaml) to see how to structure your configuration file. Default value is `passive`. 
+ `config_file`: The name of the config file you want to use. It must be in the ROS2 workspace under this package's `config` folder. Default value is `default_passive.yaml`. 

### Transform Node Launch

Launch parameters:
+ `tag_topic`: This is the topic you want remapped to the transform node's `input/tag_position` topic. The default happens to be one of the tags used in the HIVE lab, `output/DW5188`.

## Parameter Files

All parameter files should be contained in the `config` directory.
At this time, if you want to make a custom deployment, you need to create a new `yaml` file that has your desired namespace instead of `default`.
Then, you invoke your launch file with
```
ros2 launch active_node.launch.py tag_namespace:=<custom name> config_file:=<custom config yaml>
```
