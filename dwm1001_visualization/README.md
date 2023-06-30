# DWM1001 Visualization Package

This is a ROS 2 package for visualizing different DWM1001 network components, such as anchors.

# Nodes

## Anchor Visualizer (`anchor_visualizer`)

This node publishes ROS 2 markers, each representing a different DWM1001 anchor. It represents the initiator anchor
with a cube and non-initiator anchors with spheres.

## Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/anchor_markers` | `visualization_msgs/MarkerArray` | Once | Visual representations of DWM1001 anchors. The initiator anchor uses a cube, and non-initiator anchors use spheres. This publisher will run only once, after startup. |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/initiator_anchor_position` | `string` | `''` | No | No | Initiator anchor position (in meters) for the DWM1001 network. Expected format is `x,y,z` |
| `~/anchor_positions` | `string` | `''` | No | No | Non-initiator anchor positions (in meters) for the DWM1001 network. Expected format is `x,y,z;x,y,z;...;x,y,z` |

### Services

This node does not provide services.

### Actions

This node does not provide actions.
