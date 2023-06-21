# DWM1001 Driver Package

This is a ROS 2 package for interfacing with the Qorvo (previously Decawave) DWM1001.

# Nodes

## Listener Node

This node connects to a DWM1001 configured as a passive anchor. It listens for position reports from tags and publishes
any that it discovers.

### Subscribers

This node does not provide subscribers.

### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/tag_position` | `dwm1001_interfaces/TagPosition` | 10 Hz | Discovered tag positions |

### Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
|-------|-----------|---------------|----------|-----------|-------------|
| `~/serial_port` | `string` | `''` | Yes | Yes | Serial port for interfacing with the DWM1001 device. |

### Services

This node does not provide services.

### Actions

This node does not provide actions.
