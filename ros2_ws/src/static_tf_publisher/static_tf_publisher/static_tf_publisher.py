#!/usr/bin/env python3

import math
from collections import defaultdict

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__(
            "static_tf_publisher",
            automatically_declare_parameters_from_overrides=True,
        )

        self.broadcaster = StaticTransformBroadcaster(self)

        sensor_params = self.get_parameters_by_prefix("sensor_transforms")

        if not sensor_params:
            self.get_logger().error("No parameters found under 'sensor_transforms'")
            return

        transforms = defaultdict(dict)

        for param_name, param in sensor_params.items():
            # Example param_name:
            # esr.translation
            # esr.rotation_rpy
            # esr.parent_frame

            parts = param_name.split(".")

            if len(parts) != 2:
                self.get_logger().warn(
                    f"Skipping unexpected parameter: sensor_transforms.{param_name}"
                )
                continue

            child_frame = parts[0]
            field_name = parts[1]

            transforms[child_frame][field_name] = param.value

        tf_msgs = []

        for child_frame, config in transforms.items():
            required_fields = ["parent_frame", "translation", "rotation_rpy"]

            missing_fields = [
                field for field in required_fields if field not in config
            ]

            if missing_fields:
                self.get_logger().error(
                    f"Skipping '{child_frame}', missing fields: {missing_fields}"
                )
                continue

            parent_frame = config["parent_frame"]
            translation = config["translation"]
            rotation_rpy = config["rotation_rpy"]

            if len(translation) != 3:
                self.get_logger().error(
                    f"Skipping '{child_frame}', translation must have 3 values"
                )
                continue

            if len(rotation_rpy) != 3:
                self.get_logger().error(
                    f"Skipping '{child_frame}', rotation_rpy must have 3 values"
                )
                continue

            tf_msg = TransformStamped()

            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = str(parent_frame)
            tf_msg.child_frame_id = str(child_frame)

            tf_msg.transform.translation.x = float(translation[0])
            tf_msg.transform.translation.y = float(translation[1])
            tf_msg.transform.translation.z = float(translation[2])

            roll = float(rotation_rpy[0])
            pitch = float(rotation_rpy[1])
            yaw = float(rotation_rpy[2])

            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            tf_msgs.append(tf_msg)

            self.get_logger().info(
                f"Loaded static TF: {parent_frame} -> {child_frame}"
            )

        if not tf_msgs:
            self.get_logger().error("No valid static transforms to publish")
            return

        self.broadcaster.sendTransform(tf_msgs)

        self.get_logger().info(
            f"Published {len(tf_msgs)} static transforms to /tf_static"
        )


def main(args=None):
    rclpy.init(args=args)

    node = StaticTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()