# DWM1001 Driver Package

This is a ROS 2 package for interfacing with the Qorvo (formerly Decawave) DWM1001.

# Nodes

## Listener Node (`listener`)

This node connects to a DWM1001 configured as a passive anchor. It listens for position reports from tags and publishes
any that it discovers.

### Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/<discovered_tag_id>` | `geometry_msgs/PointStamped` | 10 Hz | Discovered tag position. This node creates a publisher for each discovered DWM1001 tag, and the topic name is the tag's identifier. |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/serial_port` | `string` | `''` | Yes | Yes | Serial port for interfacing with the DWM1001 device. |

### Services

This node does not provide services.

### Actions

This node does not provide actions.


## Dummy Listener Node (`dummy_listener`)

This node imitates the `listener` node by publishing a fixed position for a single tag.

### Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/<tag_id>` | `geometry_msgs/PointStamped` | 10 Hz | Specified tag identifier from the node parameters. |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/tag_id` | `string` | `''` | Yes | Yes | Identifier for the imitated DWM1001 tag. |

### Services

This node does not provide services.

### Actions

This node does not provide actions.
