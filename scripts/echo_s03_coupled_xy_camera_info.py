#!/usr/bin/env python3
"""Opt-in, coupled XY CameraInfo relay for ECHO simulated D455 (640x480 only).
Never writes to raw camera topics; preserves source headers and all other fields.
"""
from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

SIDES = ("front", "rear", "left", "right")
BASE = "/echo/cameras"
OUT = "/echo/s03/calibration/coupled_xy"
FX = 317.04311998


def valid_model(msg, cx=320.0, cy=240.0):
    values = (msg.k[0], msg.k[4], msg.k[2], msg.k[5],
              msg.p[0], msg.p[5], msg.p[2], msg.p[6])
    target = (FX, FX, cx, cy, FX, FX, cx, cy)
    return (msg.width, msg.height) == (640, 480) and all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=1e-4) for a, b in zip(values, target))


def corrected(msg, frame):
    if not valid_model(msg) or msg.header.frame_id != frame:
        raise ValueError(f"Unexpected source CameraInfo for {frame}: {msg.header.frame_id}, K={list(msg.k)}")
    info = deepcopy(msg)
    info.k[2] = info.p[2] = 319.5
    info.k[5] = info.p[6] = 239.5
    return info


class Relay(Node):
    def __init__(self):
        super().__init__("echo_s03_coupled_xy_camera_info")
        self.pubs = []
        self.subs = []
        self.seen = set()
        for side in SIDES:
            for sensor in ("color", "depth"):
                key = f"{side}/{sensor}"
                frame = f"echo_d455_{side}_{sensor}_optical_frame"
                pub = self.create_publisher(CameraInfo, f"{OUT}/{key}/camera_info", 10)
                self.pubs.append(pub)
                self.subs.append(self.create_subscription(
                    CameraInfo, f"{BASE}/{key}/camera_info",
                    lambda msg, key=key, frame=frame, pub=pub: self.on_info(msg, key, frame, pub),
                    qos_profile_sensor_data))

    def on_info(self, msg, key, frame, pub):
        # Fail closed on an unexpected calibration, rather than passing mixed models.
        info = corrected(msg, frame)
        pub.publish(info)
        if key not in self.seen:
            self.seen.add(key)
            self.get_logger().info(f"Corrected {key}: cx=319.5 cy=239.5; source stamp retained")


def main():
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
