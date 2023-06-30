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
from geometry_msgs.msg import PointStamped, TransformStamped

import dwm1001
import serial


class PassiveTagNode(Node):
    def __init__(self) -> None:
        super().__init__("passive_tag")

        self._declare_parameters()

        serial_handle = self._open_serial_port(self.get_parameter("serial_port").value)
        self.dwm_handle = dwm1001.PassiveTag(serial_handle)

        self.dwm_handle.start_position_reporting()
        self.get_logger().info("Started position reporting.")

        if self.get_parameter("ignore_tags").value:
            self.tags_to_ignore = self.get_parameter("ignore_tags").value.split(",")
        else:
            self.tags_to_ignore = []

        if self.tags_to_ignore:
            self.get_logger().info(
                f"Ignoring tags: "
                + ", ".join(f"'{tag}'" for tag in self.tags_to_ignore)
            )

        self.publishers_dict = dict()
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

        ignore_tags_descriptor = ParameterDescriptor(
            description="DWM1001 tags to ignore. This node will not publish "
            + "position reports coming from these tags. Expected format is "
            + " comma-separated values. Example: 'DW1234,DW1235,DW1236'.",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        self.declare_parameter("ignore_tags", "", ignore_tags_descriptor)

    def timer_callback(self):
        try:
            tag_id, tag_position = self.dwm_handle.wait_for_position_report()
        except dwm1001.ParsingError:
            self.get_logger().warn("Could not parse position report. Skipping it.")
            return

        if "DW" + tag_id in self.tags_to_ignore:
            return

        time_stamp = self.get_clock().now().to_msg()

        msg = PointStamped()

        msg.header.stamp = time_stamp
        msg.header.frame_id = "dwm1001"

        msg.point.x = tag_position.x_m
        msg.point.y = tag_position.y_m
        msg.point.z = tag_position.z_m

        if tag_id not in self.publishers_dict:
            self.get_logger().info(
                f"Discovered new active tag 'DW{tag_id}'. Creating publisher."
            )

            self.publishers_dict[tag_id] = self.create_publisher(
                PointStamped, "dw" + tag_id, 1
            )

        self.publishers_dict[tag_id].publish(msg)

        # DWM1001 tags publish at 10 Hz, so we want 2 times that
        # (Nyquist theorem) per known tag.
        self.timer.timer_period_ns = 1 / (20 * len(self.publishers_dict)) * 1e9


def main(args=None):
    rclpy.init(args=args)

    passive_tag = PassiveTagNode()
    rclpy.spin(passive_tag)

    passive_tag.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
