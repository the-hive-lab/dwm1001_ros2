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

from geometry_msgs.msg import Point, PointStamped, Quaternion, Transform, Vector3
from nav_msgs.msg import Odometry

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from scipy.spatial.transform import Rotation


def quaternion_msg_to_rotation_matrix(quaternion: Quaternion) -> np.ndarray:
    quat = np.array(
        [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]
    )

    return Rotation.from_quat(quat).as_matrix()


def vector3_msg_to_vector(vector3: Vector3) -> np.ndarray:
    return np.asarray(
        [
            [vector3.x],
            [vector3.y],
            [vector3.z],
        ]
    )


def point_msg_to_augmented_vector(point: Point) -> np.ndarray:
    return np.array([[point.x], [point.y], [point.z], [1]])


def augmented_vector_to_point_msg(vector: np.ndarray) -> Point:
    point = Point()

    point.x = vector[0, 0]
    point.y = vector[1, 0]
    point.z = vector[2, 0]

    return point


def transform_msg_to_matrix(transform: Transform) -> np.ndarray:
    rotation_matrix = quaternion_msg_to_rotation_matrix(transform.rotation)
    translation_vector = vector3_msg_to_vector(transform.translation)

    transform_matrix = np.hstack([rotation_matrix, translation_vector])
    transform_matrix = np.vstack([transform_matrix, np.array([0, 0, 0, 1])])

    return transform_matrix


class Dwm1001TransformNode(Node):
    def __init__(self) -> None:
        super().__init__("dwm1001_transform")

        self.tag_position_sub = self.create_subscription(
            PointStamped, "tag_position", self._tag_position_callback, 1
        )
        self.odom_pub = self.create_publisher(Odometry, "odometry/ips", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _tag_position_callback(self, msg: PointStamped) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "dwm1001", rclpy.time.Time()
            )
        except TransformException:
            self.get_logger().error(
                "Could not transform from 'dwm1001' to 'map'. Skipping transform."
            )
            return

        transform_matrix = transform_msg_to_matrix(transform.transform)
        dwm1001_point = point_msg_to_augmented_vector(msg.point)
        map_point = transform_matrix @ dwm1001_point

        map_odom = Odometry()
        map_odom.header.stamp = self.get_clock().now().to_msg()
        map_odom.header.frame_id = "map"
        map_odom.pose.pose.position = augmented_vector_to_point_msg(map_point)
        map_odom.twist.covariance[0] = -1

        self.odom_pub.publish(map_odom)


def main(args=None) -> None:
    rclpy.init(args=args)

    dwm1001_transform = Dwm1001TransformNode()
    rclpy.spin(dwm1001_transform)

    dwm1001_transform.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
