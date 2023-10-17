# DWM1001 Transform Package

This is a ROS 2 package for transforming points from the `dwm1001` frame to the `map` frame. The package and its interfaces were inspired by the `navsat_transform_node` from the [`robot_localization`](https://index.ros.org/p/robot_localization/) package.

# Nodes

## DWM1001 Transform Node (`dwm1001_transform`)

This node transforms incoming DWM1001 tag locations (`PointStamped` messages) from the common `dwm1001` frame to the `map` frame.

### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `~/input/tag_position` | `geometry_msgs/PointStamped` | DWM1001 tag's position in the `dwm1001` frame. |
| `~/tf` | `tf2_msgs/TFMessage` | Transform from the `dwm1001` frame to the `map` frame. |
| `~/tf_static` | `tf2_msgs/TFMessage` | Transform from the `dwm1001` frame to the `map` frame. |

**Note:** The transform between the `dwm1001` frame and the `map` frame can be static or dynamic as long as it's provided.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/output/odometry/ips` | `nav_msgs/Odometry` | Event-driven | DWM1001 tag's position transformed to the `map` frame. The following message fields are unused: `child_frame_id`, `pose.pose.orientation`, and `twist`. |

### Parameters

This node does not provide parameters.

### Services

This node does not provide services.

### Actions

This node does not provide actions.
