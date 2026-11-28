#!/usr/bin/env python3
"""Passive S04-1B preflight. Exits nonzero on any failed gate."""
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def main():
    directory = Path(sys.argv[1]).resolve(strict=True)
    rclpy.init()
    node = rclpy.create_node('echo_s04_obstacle_preflight')
    buf = Buffer()
    listener = TransformListener(buf, node)
    clocks = deque(maxlen=50)
    scans = deque(maxlen=8)
    backwards = [False]

    def on_clock(msg):
        value = stamp(msg.clock)
        if clocks and value < clocks[-1] - 1e-6:
            backwards[0] = True
        clocks.append(value)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    clients = {n: node.create_client(GetState, f'/{n}/get_state') for n in ('controller_server', 'collision_monitor', 'bt_navigator')}
    futures = {}
    try:
        deadline = time.monotonic() + 10
        ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for n, client in clients.items():
                if n not in futures and client.service_is_ready():
                    futures[n] = client.call_async(GetState.Request())
            if (len(clocks) >= 2 and clocks[-1] > clocks[0] and len(scans) >= 2
                    and len(futures) == len(clients) and all(f.done() for f in futures.values())
                    and buf.can_transform('base_link', scans[-1].header.frame_id, Time())):
                ready = True
                break
        require(ready, 'No complete live clock/scan/TF/lifecycle data within 10 wall seconds')
        require(not backwards[0], 'Clock moved backwards during preflight')
        for n, f in futures.items():
            require(f.result().current_state.id == 3, f'{n} is not active')
        pubs = [(p.node_namespace, p.node_name) for p in node.get_publishers_info_by_topic('/cmd_vel')]
        require(pubs == [('/', 'collision_monitor')], f'Unexpected final velocity publishers: {pubs}')
        scan = scans[-1]
        require(stamp(scan.header.stamp) > stamp(scans[0].header.stamp), 'Scan timestamps did not advance')
        tf = buf.lookup_transform('base_link', scan.header.frame_id, Time()).transform
        t, q = tf.translation, tf.rotation
        require(abs(t.x + .232) < .02 and abs(t.y) < .02 and abs(t.z - .526) < .02 and max(abs(q.x), abs(q.y), abs(q.z)) < .01, 'Unexpected LiDAR transform')
        with (directory / 'physics.jsonl').open() as f:
            lines = deque(f, maxlen=30)
        rows = []
        for line in lines:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass
        require(not any(r.get('type') == 'error' for r in rows), 'Fixture logger reported an error')
        poses = [r for r in rows if r.get('type') == 'pose']
        require(bool(poses), 'No GT poses')
        gt = poses[-1]
        gap = gt['projected_clearance_m']
        gt_age = (time.time_ns() - gt['wall_utc_ns']) / 1e9
        scan_age = clocks[-1] - stamp(scan.header.stamp)
        gt_sim_delta = clocks[-1] - gt['sim_time_s']
        points = []
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                a = scan.angle_min + i * scan.angle_increment
                points.append((r * math.cos(a) + t.x, r * math.sin(a) + t.y))
        hits = sum(abs(x - (gap + .12)) <= .06 and abs(y) <= .30 for x, y in points)
        zone = sum(-.82 <= x <= .40 and -.40 <= y <= .40 for x, y in points)
        print(f'Estimated clearance: {gap:.4f} m')
        print(f'Box-compatible scan points: {hits}; StopZone points: {zone}')
        print(f'Scan age (sim): {scan_age:.4f} s; GT age (wall): {gt_age:.4f} s')
        print(f'Clock minus GT simulation time: {gt_sim_delta:.4f} s')
        require(.35 <= gap <= .50 and hits >= 4 and zone < 4 and -.2 <= scan_age <= .6 and 0 <= gt_age <= 2 and abs(gt_sim_delta) <= .6, 'Clearance, detection or freshness gate failed')
        print('PASS: passive preflight; operator authorization still required')
        return 0
    except Exception as exc:
        print(f'FAIL: {exc}; DO NOT COMMAND MOVEMENT', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
