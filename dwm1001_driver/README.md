# DWM1001 Driver Package

This is a ROS 2 package for interfacing with the Qorvo (formerly Decawave) DWM1001.

# Nodes

## Passive Tag Node (`passive_tag`)

This node connects to a DWM1001 configured as a passive tag. It listens for position reports from active tags and
publishes any that it discovers.

### Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/output/<discovered_tag_label>` | `geometry_msgs/PointStamped` | 10 Hz | Discovered active tag position. This node creates a publisher for each discovered DWM1001 active tag, and the topic name is the tag's label. |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/serial_port` | `string` | `''` | Yes | Yes | Serial port for interfacing with the DWM1001 device. |
| `~/ignore_tags` | `string` | `''` | No | Yes | Comma-separated list of DWM1001 tags to ignore. This node will not publish reports from tags on this list. Example: `DW1234,DW1235,DW1236`. |

### Services

This node does not provide services.

### Actions

This node does not provide actions.


## Dummy Passive Tag Node (`dummy_passive_tag`)

This node imitates the `passive_tag` node by publishing a fixed position for a single "discoverd" active tag.

### Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/<tag_label>` | `geometry_msgs/PointStamped` | 10 Hz | "Discoverd" active tag's current position in the common DWM1001 coordinate frame. |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/tag_label` | `string` | `''` | Yes | Yes | Label for the imitated DWM1001 tag. |

### Services

This node does not provide services.

### Actions

This node does not provide actions.


## Active Tag Node (`active_tag`)

This node connects to a DWM1001 configured as an active tag and publishes its position.

### Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/output/<tag_id>` | `geometry_msgs/PointStamped` | 10 Hz | Tag's current position in the common DWM1001 coordinate frame. |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/serial_port` | `string` | `''` | Yes | Yes | Serial port for interfacing with the DWM1001 device. |
| `~/tag_id` | `string` | `''` | Yes | Yes | The ID for the particular DWM1001 device. |


### Services

This node does not provide services.

### Actions

This node does not provide actions.


## Dummy Passive Tag Node (`dummy_active_tag`)

This node imitates the `active_tag` node by publishing a fixed position for the tag.

### Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/<tag_label>` | `geometry_msgs/PointStamped` | 10 Hz | Tag's current position in the common DWM1001 coordinate frame. |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/tag_label` | `string` | `''` | Yes | Yes | Label for the imitated DWM1001 tag. |

### Services

This node does not provide services.

### Actions

This node does not provide actions.
