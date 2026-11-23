#!/usr/bin/env python3
"""ECHO S01: publish native depth CameraInfo from verified intrinsics."""

from copy import deepcopy
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

PREFIX = "/echo/cameras/left"


class DepthCameraInfoRelay(Node):
    def __init__(self):
        super().__init__("echo_d455_left_depth_camera_info")

        self.depth_cy_mode = os.environ.get("ECHO_S03_DEPTH_CY_MODE", "baseline").strip().lower()
        if self.depth_cy_mode not in ("baseline", "corrected"):
            raise ValueError("ECHO_S03_DEPTH_CY_MODE must be baseline or corrected")
        print(f"[ECHO S03-D5] {PREFIX} depth CameraInfo mode={self.depth_cy_mode}", flush=True)
        self.calibration = None
        self.published = 0

        self.publisher = self.create_publisher(
            CameraInfo,
            PREFIX + "/depth/camera_info",
            10,  # Reliable, volatile QoS
        )

        self.rgb_info_sub = self.create_subscription(
            CameraInfo,
            PREFIX + "/color/camera_info",
            self.on_calibration,
            qos_profile_sensor_data,
        )

        self.depth_sub = self.create_subscription(
            Image,
            PREFIX + "/depth/image_raw",
            self.on_depth,
            qos_profile_sensor_data,
        )

        print("[ECHO] Depth CameraInfo relay started", flush=True)

    def on_calibration(self, msg):
        if (msg.width, msg.height) != (640, 480):
            return
        if abs(msg.k[0] - 317.04312) > 0.01:
            return
        if abs(msg.k[4] - 317.04312) > 0.01:
            return

        self.calibration = msg

    def on_depth(self, depth):
        if self.calibration is None:
            return

        if (depth.width, depth.height) != (
            self.calibration.width,
            self.calibration.height,
        ):
            return

        if depth.encoding != "32FC1":
            return

        info = deepcopy(self.calibration)
        info.header = deepcopy(depth.header)

        # Simulated renderer-native depth uses a pixel-center convention
        # consistent with cy=239.5. Opt-in only; RGB CameraInfo is untouched.
        if self.depth_cy_mode == "corrected":
            info.k[5] = 239.5
            info.p[6] = 239.5

        self.publisher.publish(info)
        self.published += 1

        if self.published == 1:
            print(
                "[PASS] Publishing depth CameraInfo",
                flush=True,
            )
            print(
                "[PASS] Frame:", info.header.frame_id,
                flush=True,
            )
            print(
                "[PASS] fx/fy:", info.k[0], info.k[4],
                flush=True,
            )


def main():
    rclpy.init()
    node = DepthCameraInfoRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
