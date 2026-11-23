#!/usr/bin/env python3
"""Single supervised NAV-0 goal 0.30 m forward in odom; preview is default."""
import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def spin_until(node, predicate, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return predicate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=float, default=0.30)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--max-wall-seconds', type=float, default=180.0)
    args = parser.parse_args()
    if not 0.05 <= args.distance <= 0.40:
        parser.error('Only a single 0.05–0.40 m test goal is allowed')

    rclpy.init()
    node = Node('echo_nav0_short_goal')
    most_recent = [None]
    node.create_subscription(Odometry, '/chassis/odom',
                             lambda msg: most_recent.__setitem__(0, msg),
                             qos_profile_sensor_data)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    accepted_handle = None
    try:
        if not spin_until(node, lambda: most_recent[0] is not None, 15.0):
            raise RuntimeError('No /chassis/odom; is Isaac Sim playing?')
        msg = most_recent[0]
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            raise RuntimeError('Unexpected odometry frames: '
                               f'{msg.header.frame_id} -> {msg.child_frame_id}')
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        pos = msg.pose.pose.position
        goal_x = pos.x + args.distance * math.cos(yaw)
        goal_y = pos.y + args.distance * math.sin(yaw)
        print(f'ODOM START ({pos.x:.3f}, {pos.y:.3f}, yaw {math.degrees(yaw):.1f} deg)')
        print(f'ODOM GOAL  ({goal_x:.3f}, {goal_y:.3f}, same yaw); '
              f'distance {args.distance:.2f} m')
        if not args.execute:
            print('PREVIEW ONLY. To send one Nav2 goal: --execute')
            return

        # DDS graph discovery is asynchronous for a newly-created rclpy node.
        # Wait for the unique velocity publisher rather than checking only once.
        expected = [('collision_monitor', '/')]
        graph_deadline = time.monotonic() + 12.0
        stable_since = None
        pubs = []
        found = []
        while rclpy.ok() and time.monotonic() < graph_deadline:
            pubs = node.get_publishers_info_by_topic('/cmd_vel')
            found = sorted((p.node_name, p.node_namespace) for p in pubs)
            if found == expected:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 1.0:
                    break
            else:
                stable_since = None
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            raise RuntimeError(
                'Expected collision_monitor as the sole /cmd_vel publisher; '
                f'after waiting, found: {found}. No goal sent.'
            )
        print('NAV0 safety graph: collision_monitor is sole /cmd_vel publisher')
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/navigate_to_pose action server unavailable')
        answer = input('Check clear space and Isaac Sim Play; type GO to execute: ').strip()
        if answer != 'GO':
            print('CANCELLED by operator before sending goal')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'odom'
        goal.pose.header.stamp = most_recent[0].header.stamp
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = q

        sent = client.send_goal_async(goal)
        if not spin_until(node, lambda: sent.done(), 15.0):
            raise RuntimeError('Nav2 did not acknowledge goal request')
        accepted_handle = sent.result()
        if not accepted_handle.accepted:
            raise RuntimeError('Nav2 rejected goal')
        print('NAV0 GOAL ACCEPTED: press Ctrl+C to cancel; stop simulation if unsafe.')
        result_future = accepted_handle.get_result_async()
        wall_start = time.monotonic()
        last_report = 0.0
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = time.monotonic() - wall_start
            if elapsed - last_report >= 5.0:
                last_report = elapsed
                current = most_recent[0]
                if current is not None:
                    p = current.pose.pose.position
                    print(f'elapsed_wall={elapsed:.1f}s odom=({p.x:.3f},{p.y:.3f})', flush=True)
            if elapsed > args.max_wall_seconds:
                print('NAV0 WALL WATCHDOG: cancelling Nav2 goal')
                cancel_future = accepted_handle.cancel_goal_async()
                if not spin_until(node, lambda: cancel_future.done(), 5.0):
                    print('NAV0 WARNING: cancellation acknowledgement timed out; stop Isaac Sim manually.')
                return
        result = result_future.result()
        print('NAV0 RESULT:', 'SUCCEEDED' if result.status == GoalStatus.STATUS_SUCCEEDED
              else f'NON-SUCCESS status={result.status}')
    except KeyboardInterrupt:
        print('NAV0 CTRL+C: requesting action cancellation')
        if accepted_handle is not None:
            cancel_future = accepted_handle.cancel_goal_async()
            if not spin_until(node, lambda: cancel_future.done(), 5.0):
                print('NAV0 WARNING: cancellation acknowledgement timed out.')
        print('If robot is not stopped, press Stop in Isaac Sim immediately.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
