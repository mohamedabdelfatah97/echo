#!/usr/bin/env python3

import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def main():
    source = Path(sys.argv[1])
    with source.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise RuntimeError("Odometry CSV is empty")

    path = RosPath()
    path.header.frame_id = "odom"

    for row in rows:
        pose = PoseStamped()
        pose.header.frame_id = "odom"

        pose.pose.position.x = float(row["x_m"])
        pose.pose.position.y = float(row["y_m"])
        pose.pose.position.z = float(row["z_m"])

        yaw = float(row["yaw_rad"])
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        path.poses.append(pose)

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )

    rclpy.init()
    node = rclpy.create_node("echo_s02_saved_path_replay")

    pub = node.create_publisher(
        RosPath,
        "/echo/s02/replay/odom_path",
        qos
    )

    print("=== ECHO S02 TRAJECTORY REPLAY ===", flush=True)
    print("Source:", source, flush=True)
    print("Frame: odom", flush=True)
    print("Recorded poses:", len(path.poses), flush=True)
    print("Topic: /echo/s02/replay/odom_path", flush=True)
    print("No robot commands will be published.", flush=True)
    print("Press Ctrl+C to stop.", flush=True)

    try:
        while rclpy.ok():
            pub.publish(path)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
