# Copyright 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from dwm1001_interfaces.msg import TagPosition

import dwm1001
import serial


class ListenerNode(Node):
    def __init__(self) -> None:
        super().__init__("listener")

        self._declare_parameters()

        serial_handle = self._open_serial_port(self.get_parameter("serial_port").value)
        self.dwm_handle = dwm1001.Listener(serial_handle)

        self.dwm_handle.start_position_reporting()
        self.get_logger().info("Started position reporting.")

        self.publisher = self.create_publisher(TagPosition, "tag_position", 1)
        self.timer = self.create_timer(1 / 10, self.timer_callback)

    def _open_serial_port(self, serial_port: str) -> serial.Serial:
        if not serial_port:
            self._shutdown_fatal("No serial port specified.")

        try:
            serial_handle = serial.Serial(serial_port, baudrate=115_200)
        except serial.SerialException:
            self._shutdown_fatal(f"Could not open serial port '{serial_port}'.")

        self.get_logger().info(f"Opened serial port: '{serial_port}'.")

        return serial_handle

    def _shutdown_fatal(self, message: str) -> None:
        self.get_logger().fatal(message + " Shutting down.")
        exit()

    def _declare_parameters(self):
        serial_port_descriptor = ParameterDescriptor(
            description="Device file or COM port associated with DWM1001",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        self.declare_parameter("serial_port", "", serial_port_descriptor)

    def timer_callback(self):
        tag_id, tag_position = self.dwm_handle.wait_for_position_report()

        msg = TagPosition()
        msg.tag_id = tag_id
        msg.position.x = tag_position.x_m
        msg.position.y = tag_position.y_m
        msg.position.z = tag_position.z_m
        msg.quality = tag_position.quality

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    listener = ListenerNode()
    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
