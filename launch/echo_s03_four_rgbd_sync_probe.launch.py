"""ECHO S03: four-camera RGB-D synchronization probe (no SLAM / no DB).

Requires the existing live four-D455 Isaac Sim stack and camera-info helpers.
Each synthetic D455's depth is registered into its RGB optical frame, then
rgbd_sync creates one RGBDImage per camera. rgbdx_sync aggregates the four.
This is intentionally NOT a mapping launch.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

SIDES = ("front", "rear", "left", "right")
BASE = "/echo/cameras"
PROBE = "/echo/s03"


def generate_launch_description():
    nodes = []
    for side in SIDES:
        prefix = f"{BASE}/{side}"
        nodes.append(
            Node(
                package="depth_image_proc",
                executable="register_node",
                name=f"echo_s03_{side}_depth_register",
                output="screen",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    ("depth/image_rect", f"{prefix}/depth/image_raw"),
                    ("depth/camera_info", f"{prefix}/depth/camera_info"),
                    ("rgb/camera_info", f"{prefix}/color/camera_info"),
                    (
                        "depth_registered/image_rect",
                        f"{PROBE}/{side}/depth_registered/image_rect",
                    ),
                ],
            )
        )
        nodes.append(
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name=f"echo_s03_{side}_rgbd_sync",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "approx_sync": False,
                        "qos": 2,
                        "topic_queue_size": 20,
                        "sync_queue_size": 20,
                    }
                ],
                remappings=[
                    ("rgb/image", f"{prefix}/color/image_raw"),
                    ("rgb/camera_info", f"{prefix}/color/camera_info"),
                    (
                        "depth/image",
                        f"{PROBE}/{side}/depth_registered/image_rect",
                    ),
                    ("rgbd_image", f"{PROBE}/{side}/rgbd_image"),
                ],
            )
        )

    nodes.append(
        Node(
            package="rtabmap_sync",
            executable="rgbdx_sync",
            name="echo_s03_rgbdx_sync",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "rgbd_cameras": 4,
                    "approx_sync": True,
                    "approx_sync_max_interval": 0.05,
                    "qos": 2,
                    "topic_queue_size": 20,
                    "sync_queue_size": 20,
                }
            ],
            remappings=[
                (f"rgbd_image{i}", f"{PROBE}/{side}/rgbd_image")
                for i, side in enumerate(SIDES)
            ]
            + [("rgbd_images", f"{PROBE}/rgbd_images")],
        )
    )

    return LaunchDescription(nodes)
