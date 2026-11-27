#!/usr/bin/env python3
"""Passive ROS Jazzy D7 logger. Ctrl+C finalizes the JSONL; publishes nothing."""
import json
from pathlib import Path
import time
from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, TwistStamped

ROOT = Path.home() / 'Downloads' / 'echo_s03_d7_trial2'

def stamp(t):
    return int(t.sec)*1_000_000_000 + int(t.nanosec)

def xyz(p):
    return [p.x, p.y, p.z]

def quat(q):
    return [q.w, q.x, q.y, q.z]

def twist(t):
    return {'linear': xyz(t.linear), 'angular': xyz(t.angular)}

class Recorder(Node):
    def __init__(self, handle):
        super().__init__('echo_s03_d7_passive_ros_logger')
        self.log_file = handle
        self.counts = Counter()
        self.clock_ns = None
        self.first_clock_ns = None
        self.last_report = -5.0
        self.subs = []
        self.add(Odometry, '/chassis/odom', self.odom)
        self.add(Clock, '/clock', self.clock)
        self.add(TFMessage, '/tf', lambda m: self.tf(m, '/tf'))
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.add(TFMessage, '/tf_static', lambda m: self.tf(m, '/tf_static'), static_qos)

    def add(self, cls, topic, callback, qos=qos_profile_sensor_data):
        self.subs.append(self.create_subscription(cls, topic, callback, qos))

    def record(self, typ, **fields):
        row = dict(type=typ, wall_monotonic_s=time.monotonic(), wall_utc_ns=time.time_ns(), **fields)
        self.log_file.write(json.dumps(row, allow_nan=False) + '\n')
        self.counts[typ] += 1

    def odom(self, m):
        p = m.pose.pose
        self.record('odom', stamp_ns=stamp(m.header.stamp), frame_id=m.header.frame_id,
                    child_frame_id=m.child_frame_id, xyz_m=xyz(p.position),
                    quat_wxyz=quat(p.orientation), twist=twist(m.twist.twist),
                    pose_covariance=list(m.pose.covariance), twist_covariance=list(m.twist.covariance),
                    latest_clock_ns=self.clock_ns)

    def clock(self, m):
        self.clock_ns = stamp(m.clock)
        if self.first_clock_ns is None:
            self.first_clock_ns = self.clock_ns
        self.record('clock', stamp_ns=self.clock_ns)
        elapsed = (self.clock_ns-self.first_clock_ns)/1e9
        if elapsed-self.last_report >= 5:
            self.last_report = elapsed
            print(f'[D7] ROS clock elapsed={elapsed:.2f} sim s; odom samples={self.counts["odom"]}', flush=True)
            self.log_file.flush()

    def tf(self, m, topic):
        self.record('tf', topic=topic, transforms=[dict(stamp_ns=stamp(t.header.stamp),
                    parent=t.header.frame_id, child=t.child_frame_id,
                    xyz_m=xyz(t.transform.translation), quat_wxyz=quat(t.transform.rotation))
                    for t in m.transforms])

    def cmd(self, m):
        t = m.twist if isinstance(m, TwistStamped) else m
        self.record('cmd_vel', stamp_ns=stamp(m.header.stamp) if isinstance(m, TwistStamped) else None,
                    latest_clock_ns=self.clock_ns, twist=twist(t))

    def graph(self):
        for topic in ('/clock', '/chassis/odom', '/cmd_vel', '/tf', '/tf_static'):
            self.record('graph', topic=topic, publishers=[dict(name=p.node_name,
                namespace=p.node_namespace, topic_type=p.topic_type) for p in self.get_publishers_info_by_topic(topic)])


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    path = ROOT / 'ros.jsonl'
    with path.open('x', encoding='utf-8', buffering=1) as f:
        rclpy.init()
        n = Recorder(f)
        try:
            # Discover actual cmd_vel type, no additional velocity publisher.
            deadline = time.monotonic()+15
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(n, timeout_sec=0.1)
                types = dict(n.get_topic_names_and_types()).get('/cmd_vel', [])
                if n.counts['odom'] and n.counts['clock'] and len(types)==1 and types[0] in ('geometry_msgs/msg/Twist','geometry_msgs/msg/TwistStamped'):
                    n.add(get_message(types[0]), '/cmd_vel', n.cmd)
                    n.graph()
                    print('[D7] READY: odometry, clock and cmd_vel subscription established.', flush=True)
                    print('[D7] Recording:', path, '| Ctrl+C to finish after the goal.', flush=True)
                    ready_clock = n.clock_ns
                    break
            else:
                n.graph()
                raise RuntimeError('No odom/clock or unique supported cmd_vel type; check Play and NAV-0.')
            announced=False
            wall_end=time.monotonic()+360
            while rclpy.ok() and time.monotonic()<wall_end:
                rclpy.spin_once(n, timeout_sec=0.1)
                if not announced and n.clock_ns is not None and n.clock_ns-ready_clock>=10_000_000_000:
                    print('[D7] 10 simulation seconds since READY. Stationary interval captured; goal can be sent if clear.', flush=True)
                    announced=True
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            f.write(json.dumps(dict(type='summary', counts=dict(n.counts)))+'\n')
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print('[D7] SAVED:', path, 'counts:', dict(n.counts), flush=True)

if __name__ == '__main__':
    main()
