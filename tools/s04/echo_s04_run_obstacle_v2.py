#!/usr/bin/env python3
"""One-terminal, supervised ECHO S04-1B1 trial. No repository edits or USD saves."""
import argparse
import json
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from collections import deque
from pathlib import Path

ROOT = Path.home() / 'echo'
DOWNLOADS = Path.home() / 'Downloads'
DEFAULT_CONFIG = DOWNLOADS / 'echo_s04_1_nav0_20261127_234442_3PvpnX/echo_s04_nav0_params.yaml'
TOPICS = ['/clock', '/chassis/odom', '/tf', '/tf_static', '/scan', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/collision_monitor_state']


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read_log(path):
    return path.read_text(errors='replace') if path.exists() else ''


def latest_gt(directory):
    with (directory / 'physics.jsonl').open() as f:
        lines = deque(f, maxlen=30)
    rows = []
    for line in lines:
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass  # Last line may be in the process of being written.
    require(not any(r.get('type') == 'error' for r in rows), 'Fixture logger reported error/reset')
    poses = [r for r in rows if r.get('type') == 'pose']
    require(bool(poses), 'No GT poses')
    return poses[-1]


def gt_guard(directory, min_gap):
    gt = latest_gt(directory)
    age = (time.time_ns() - gt['wall_utc_ns']) / 1e9
    require(0 <= age <= 2, f'Stale ground truth: {age:.2f} s')
    gap = gt['projected_clearance_m']
    require(gap >= min_gap, f'Estimated clearance {gap:.3f} m below guard {min_gap:.2f} m')
    return gt


def preflight(directory):
    # Imports deferred: orchestration does not own a ROS node.
    import math
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
    from lifecycle_msgs.srv import GetState

    def stamp(s):
        return s.sec + s.nanosec * 1e-9

    rclpy.init(args=[])
    node = rclpy.create_node('echo_s04_obstacle_runner_preflight')
    buf = Buffer()
    listener = TransformListener(buf, node)
    clocks, scans = deque(maxlen=50), deque(maxlen=8)
    backwards = [False]

    def on_clock(msg):
        value = stamp(msg.clock)
        if clocks and value < clocks[-1] - 1e-6:
            backwards[0] = True
        clocks.append(value)

    def on_scan(msg):
        scans.append(msg)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
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
        gt = gt_guard(directory, .35)
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
        print('PASS: passive preflight')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def check_local_processes():
    names = {'controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor', 'lifecycle_manager', 'pointcloud_to_laserscan_node'}
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
        except OSError:
            continue
        if argv and (Path(argv[0]).name in names or ('bag' in argv and 'record' in argv and any(Path(a).name == 'ros2' for a in argv))):
            found.append((entry.name, ' '.join(argv)[:180]))
    require(not found, f'Existing navigation/recording processes; close them normally first: {found}')


def run_trial(config):
    import yaml
    require(os.environ.get('ROS_DISTRO') == 'jazzy', 'Source ROS Jazzy and Isaac workspace first')
    require(os.environ.get('ROS_DOMAIN_ID') == '0' and os.environ.get('RMW_IMPLEMENTATION') == 'rmw_fastrtps_cpp', 'Unexpected ROS environment')
    check_local_processes()
    supervisor = ROOT / 'scripts/start_echo_dynamic_four_d455.py'
    goal_script = ROOT / 'scripts/echo_nav0_short_goal.py'
    launch = ROOT / 'launch/echo_nav0.launch.py'
    for p in (supervisor, goal_script, launch, ROOT / 'tools/s04/obstacle_fixture.py', config):
        require(p.is_file(), f'Missing {p}')
    original = yaml.safe_load(config.read_text())
    nav = original['bt_navigator']['ros__parameters']
    bt = Path(nav['default_nav_to_pose_bt_xml'])
    require(bt.is_file(), f'Missing BT: {bt}')
    import xml.etree.ElementTree as ET
    tree = ET.parse(bt)
    require(not any(x.tag in {'Spin','BackUp','RecoveryNode','DriveOnHeading'} for x in tree.iter()), 'Unexpected motion recovery in BT')
    require(nav['navigators'] == ['navigate_to_pose'], 'Unexpected navigator config')
    baseline = yaml.safe_load((ROOT/'config/nav2/nav0/echo_nav0_params.yaml').read_text())
    require(all(original[k] == baseline[k] for k in ('collision_monitor','velocity_smoother','controller_server','local_costmap','global_costmap','planner_server','behavior_server')), 'NAV-0 safety/control parameters differ from baseline')
    env = dict(os.environ, ECHO_S03_DEPTH_CY_MODE='baseline', PYTHONUNBUFFERED='1')
    env.pop('ECHO_S04_OBSTACLE_DIR', None)
    sim_cmd = [sys.executable, '-u', str(supervisor), '--calibration', 'coupled_xy', '--agents', 'none']
    check = subprocess.run(sim_cmd + ['--check'], cwd=ROOT, env=env, capture_output=True, text=True, timeout=20)
    print(check.stdout, end='', flush=True)
    require(check.returncode == 0 and 'Existing experiment processes: 0' in check.stdout, f'Supervisor preflight rejected startup: {check.stderr}')
    directory = Path(tempfile.mkdtemp(prefix='echo_s04_1b1_auto_'+time.strftime('%Y%m%d_%H%M%S')+'_', dir=DOWNLOADS))
    env['ECHO_S04_OBSTACLE_DIR'] = str(directory)
    shutil.copy2(config, directory/'input_nav0_params.yaml')
    shutil.copy2(bt, directory/'input_behavior_tree.xml')
    (directory/'repo_state.txt').write_text(subprocess.check_output(['git','status','--short','--branch'], cwd=ROOT, text=True)+subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True))
    print(f'\nTRIAL: {directory}', flush=True)
    processes = {}
    outcome = 'not executed'
    goal_started = False
    emergency = False

    def start(name, cmd, stdin=None):
        log = directory / (name + '.log')
        with log.open('x') as out:
            proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdin=stdin if stdin is not None else subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        processes[name] = proc
        return proc

    def assert_alive(names):
        for n in names:
            require(processes[n].poll() is None, f'{n} exited; see {directory/n}.log')

    def wait_text(name, text, seconds, names):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert_alive(names)
            if text in read_log(directory/(name+'.log')):
                return
            time.sleep(.15)
        raise RuntimeError(f'Timeout waiting for {name}: {text}')

    def send(name, sig):
        p = processes.get(name)
        if p is not None and p.poll() is None:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass

    def stop(name, timeout=10):
        p = processes.get(name)
        if p is None or p.poll() is not None:
            return
        send(name, signal.SIGINT)
        try:
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            send(name, signal.SIGTERM)
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                # Supervisor owns independent child groups; do not kill it blindly.
                print(f'WARNING: {name} has not stopped; inspect PID {p.pid} and log manually', flush=True)

    try:
        start('supervisor', sim_cmd)
        print('Starting simulator. Leave Isaac Sim NOT PLAYING until instructed.', flush=True)
        wait_text('supervisor', 'Press Play in Isaac Sim', 210, ['supervisor'])
        require((directory/'fixture.json').is_file(), 'Fixture metadata missing')
        start('nav0', ['ros2','launch',str(launch),f'params_file:={config}'])
        start('recorder', ['ros2','bag','record','-s','mcap','-o',str(directory/'bag'),'--disable-keyboard-controls','--topics',*TOPICS])
        wait_text('recorder', 'Recording...', 25, ['supervisor','nav0','recorder'])
        print('\nREADY: simulator, NAV-0 and recorder started.\nInspect the orange box and surrounding clearance visually.\nIf safe, press Play ONCE in Isaac Sim, then return to this terminal.\nDo not pause/reset during this trial. Ctrl+C aborts; pause the simulator immediately if unsafe.', flush=True)
        wait_text('nav0', 'Managed nodes are active', 120, ['supervisor','nav0','recorder'])
        wait_text('recorder', 'All requested topics are subscribed', 20, ['supervisor','nav0','recorder'])
        print('\nNAV-0 and recording ready. Type GO promptly to authorize one 0.40 m forward goal. Anything else aborts.', flush=True)
        print('Authorization (20 second limit): ', end='', flush=True)
        readable, _, _ = select.select([sys.stdin], [], [], 20)
        require(bool(readable), 'Authorization timed out; no goal sent')
        require(sys.stdin.readline().strip() == 'GO', 'Operator declined')
        assert_alive(['supervisor','nav0','recorder'])
        print('Running live safety preflight...', flush=True)
        pf = start('preflight', [sys.executable,'-u',str(Path(__file__).resolve()),'--internal-preflight',str(directory)])
        pf.wait(timeout=15)
        print(read_log(directory/'preflight.log'), end='', flush=True)
        require(pf.returncode == 0, 'Preflight rejected movement')
        assert_alive(['supervisor','nav0','recorder'])
        gt_guard(directory,.35)
        # Explicit GO above authorizes exactly this one invocation. Existing helper still checks DDS ownership.
        goal = start('goal', [sys.executable,'-u',str(goal_script),'--distance','0.40','--execute','--max-wall-seconds','15'], subprocess.PIPE)
        goal_started = True
        goal.stdin.write(b'GO\n')
        goal.stdin.close()
        deadline = time.monotonic()+50
        printed = 0
        last_sim = None
        last_advance = time.monotonic()
        while goal.poll() is None:
            assert_alive(['supervisor','nav0','recorder'])
            log = read_log(directory/'goal.log')
            if len(log)>printed:
                print(log[printed:],end='',flush=True)
                printed=len(log)
            accepted = 'NAV0 GOAL ACCEPTED' in log
            gt = gt_guard(directory,.22 if accepted else .35)
            if last_sim is not None:
                require(gt['sim_time_s'] >= last_sim, 'GT time reversed')
            if last_sim is None or gt['sim_time_s'] > last_sim:
                last_advance=time.monotonic()
            last_sim=gt['sim_time_s']
            require(time.monotonic()-last_advance <= 2,'GT simulation time stopped advancing')
            require(time.monotonic()<deadline,'Outer motion watchdog expired')
            time.sleep(.10)
        log = read_log(directory/'goal.log')
        print(log[printed:],end='',flush=True)
        require(goal.returncode == 0, f'Goal helper exit {goal.returncode}')
        # The helper can return without a confirmed result (watchdog/cancellation); do not label it a pass.
        outcome = next((line for line in reversed(log.splitlines()) if 'NAV0 RESULT:' in line), 'No confirmed action result; inspect goal log')
        require('NAV0 RESULT:' in outcome, outcome)
        for _ in range(20):
            assert_alive(['supervisor','nav0','recorder'])
            gt_guard(directory,.22)
            time.sleep(.1)
        print('Action completed. Ending recording and closing managed processes.',flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        emergency = True
        outcome = f'ABORT: {type(exc).__name__}: {exc}'
        print('\n'+outcome,flush=True)
        if goal_started:
            print('If robot is moving, pause Isaac Sim immediately. Requesting cancellation and simulator shutdown.',flush=True)
            send('goal',signal.SIGINT)
            send('supervisor',signal.SIGINT)
    finally:
        # In a motion abort, simulator shutdown has already been requested above.
        stop('goal',7)
        stop('preflight',3)
        stop('recorder',15)
        stop('nav0',12)
        stop('supervisor',20)
        logs = re.findall(r'\[ECHO\] Logs: (\S+)',read_log(directory/'supervisor.log'))
        if logs and Path(logs[-1]).is_dir():
            dest=directory/'supervisor_child_logs'
            dest.mkdir(exist_ok=True)
            for path in Path(logs[-1]).glob('*.log'):
                shutil.copy2(path,dest/path.name)
        (directory/'outcome.txt').write_text(outcome+'\n')
        if (directory/'bag'/'metadata.yaml').is_file():
            info=subprocess.run(['ros2','bag','info',str(directory/'bag')],capture_output=True,text=True,timeout=20)
            (directory/'bag_info.txt').write_text(info.stdout+info.stderr)
            print('\n'+info.stdout,flush=True)
        if (directory/'physics.jsonl').is_file():
            with (directory/'physics.jsonl').open() as f:
                print('=== LAST GROUND TRUTH RECORDS ===\n'+''.join(deque(f,maxlen=3)),flush=True)
        print(f'\nOUTCOME: {outcome}\nEVIDENCE: {directory}\nDo not restart or retry automatically.',flush=True)
    return int(emergency)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    p.add_argument('--internal-preflight',type=Path,help=argparse.SUPPRESS)
    args=p.parse_args()
    try:
        if args.internal_preflight:
            preflight(args.internal_preflight)
            return 0
        return run_trial(args.config.expanduser().resolve())
    except (Exception, KeyboardInterrupt) as exc:
        print(f'FAIL: {type(exc).__name__}: {exc}',file=sys.stderr,flush=True)
        return 1

if __name__=='__main__':
    sys.exit(main())
