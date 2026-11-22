#!/usr/bin/env python3
"""ECHO S01: static optical frames for the simulated rear D455.

Measured from the Isaac Sim USD camera hierarchy.
Parent: echo_d455_rear_link
Child frames follow ROS optical convention:
    +Z forward, +X right, +Y down.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def make_transform(node, child, x, y, z):
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id = "echo_d455_rear_link"
    tf.child_frame_id = child

    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z

    tf.transform.rotation.x = 0.5
    tf.transform.rotation.y = 0.5
    tf.transform.rotation.z = -0.5
    tf.transform.rotation.w = -0.5

    return tf


def main():
    rclpy.init()
    node = rclpy.create_node("echo_d455_rear_static_tf")
    broadcaster = StaticTransformBroadcaster(node)

    transforms = [
        make_transform(
            node,
            "echo_d455_rear_color_optical_frame",
            0.0, 0.0115, 0.0,
        ),
        make_transform(
            node,
            "echo_d455_rear_depth_optical_frame",
            0.0, 0.0, 0.0,
        ),
    ]

    broadcaster.sendTransform(transforms)

    print("[ECHO] Front D455 optical TF broadcaster started", flush=True)
    print("[ECHO] Published 2 static transforms", flush=True)
    print("[ECHO] Parent: echo_d455_rear_link", flush=True)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
