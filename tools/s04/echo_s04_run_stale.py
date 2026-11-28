#!/usr/bin/env python3
"""ECHO S04-1C: supervised simulator-only single-run scan interruption.
Requires the previously used echo_s04_run_obstacle_v2.py beside this file.
No repository writes; no obstacle fixture; no direct velocity publishing.
"""
import argparse
import copy
import json
import math
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from collections import deque

import echo_s04_run_obstacle_v2 as base

GATE = '/echo/s04/monitor_scan'
TIMEOUT = 1.5
TOPICS = base.TOPICS + [GATE]


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def stamp(msg):
    return msg.sec + msg.nanosec * 1e-9


def nonzero(msg):
    return max(abs(getattr(v, a)) for v in (msg.linear, msg.angular) for a in ('x','y','z')) > 0.001


def driver(directory):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener
    rclpy.init(args=[])
    node = Node('echo_s04_scan_gate')
    pub = node.create_publisher(LaserScan, GATE, qos_profile_sensor_data)
    buf = Buffer()
    listener = TransformListener(buf, node)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    clocks = deque(maxlen=50)
    events = deque(maxlen=5000)
    latest = {}
    forwarded = None
    dropped = False
    reversed_clock = False
    handle = None
    future = None
    start_xy = None
    log = (directory/'driver_events.jsonl').open('x', buffering=1)

    def event(kind, **kwargs):
        row = dict(kind=kind, wall_ns=time.time_ns(), sim_s=clocks[-1] if clocks else None, **kwargs)
        log.write(json.dumps(row)+'\n')
        events.append(row)
        return row

    def on_clock(msg):
        nonlocal reversed_clock
        s = stamp(msg.clock)
        if clocks and s < clocks[-1] - 1e-6:
            reversed_clock = True
        clocks.append(s)
        latest['clock_wall'] = time.monotonic()

    def on_scan(msg):
        nonlocal forwarded
        latest['scan'] = msg
        latest['scan_wall'] = time.monotonic()
        if not dropped:
            pub.publish(msg)  # Original timestamp and frame, unchanged.
            forwarded = stamp(msg.header.stamp)
            latest['forward_wall'] = time.monotonic()
        event('scan', header_s=stamp(msg.header.stamp), forwarded=not dropped)

    def on_odom(msg):
        latest['odom'] = msg
        latest['odom_wall'] = time.monotonic()

    def on_up(msg):
        latest['up'] = msg
        latest['up_wall'] = time.monotonic()
        event('upstream', nonzero=nonzero(msg), vx=msg.linear.x, wz=msg.angular.z)

    def on_final(msg):
        latest['final'] = msg
        latest['final_wall'] = time.monotonic()
        event('final', nonzero=nonzero(msg), vx=msg.linear.x, wz=msg.angular.z)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/chassis/odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel_smoothed', on_up, 20)
    node.create_subscription(Twist, '/cmd_vel', on_final, 20)
    clients = {n:node.create_client(GetState, f'/{n}/get_state') for n in ('controller_server','collision_monitor','bt_navigator')}
    futures = {}

    def spin():
        rclpy.spin_once(node, timeout_sec=.03)

    def graph():
        def publishers(topic):
            return sorted((p.node_namespace,p.node_name) for p in node.get_publishers_info_by_topic(topic))
        require(publishers('/cmd_vel') == [('/','collision_monitor')], 'Unexpected final velocity publishers')
        require(publishers('/cmd_vel_smoothed') == [('/','velocity_smoother')], 'Unexpected upstream publishers')
        require(publishers(GATE) == [('/','echo_s04_scan_gate')], 'Unexpected gate publishers')
        require(publishers('/scan') == [('/','echo_nav0_pointcloud_to_laserscan')], 'Unexpected raw scan publishers')
        subs = node.get_subscriptions_info_by_topic(GATE)
        require(any(s.node_name == 'collision_monitor' for s in subs), 'Monitor not subscribed to gate')
        require(not any(s.node_name == 'collision_monitor' for s in node.get_subscriptions_info_by_topic('/scan')), 'Monitor also consumes raw scan')

    def live():
        require(not reversed_clock, 'Simulation clock moved backwards')
        now = time.monotonic()
        for k in ('clock_wall','scan_wall','odom_wall'):
            require(k in latest and now-latest[k] < 2, f'Stale {k}')
        require(-.2 <= clocks[-1]-stamp(latest['scan'].header.stamp) <= .6, 'Raw scan not fresh')
        require(abs(clocks[-1]-stamp(latest['odom'].header.stamp)) <= .6, 'Odometry not fresh')

    def travel():
        p = latest['odom'].pose.pose.position
        return math.hypot(p.x-start_xy[0],p.y-start_xy[1])

    try:
        print('GATE READY', flush=True)
        deadline = time.monotonic()+240
        while not (directory/'authorize').exists():
            require(time.monotonic()<deadline, 'Authorization timeout')
            spin()
        deadline = time.monotonic()+10
        ready = False
        while time.monotonic()<deadline:
            spin()
            for n,c in clients.items():
                if n not in futures and c.service_is_ready():
                    futures[n] = c.call_async(GetState.Request())
            if (len(clocks)>=2 and clocks[-1]>clocks[0] and forwarded is not None and 'odom' in latest
                and len(futures)==len(clients) and all(f.done() for f in futures.values())
                and client.server_is_ready() and buf.can_transform('base_link',latest['scan'].header.frame_id,Time())):
                ready=True
                break
        require(ready,'Incomplete live preflight')
        for n,f in futures.items():
            require(f.result().current_state.id==3, f'{n} not active')
        live(); graph()
        require(-.2<=clocks[-1]-forwarded<=.6,'Relayed scan not fresh')
        scan=latest['scan']
        tf=buf.lookup_transform('base_link',scan.header.frame_id,Time()).transform
        t,q=tf.translation,tf.rotation
        require(abs(t.x+.232)<.02 and abs(t.y)<.02 and abs(t.z-.526)<.02 and max(abs(q.x),abs(q.y),abs(q.z))<.01,'Unexpected scan transform')
        zone=0; corridor=0
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                a=scan.angle_min+i*scan.angle_increment
                x,y=r*math.cos(a)+t.x,r*math.sin(a)+t.y
                zone+=(-.82<=x<=.4 and abs(y)<=.4)
                corridor+=(.12<=x<=.85 and abs(y)<=.4)
        require(zone<4 and corridor==0, f'Blocked initial scan: zone={zone}, corridor={corridor}')
        od=latest['odom']; p=od.pose.pose.position; q=od.pose.pose.orientation
        require(od.header.frame_id=='odom' and od.child_frame_id=='base_link','Unexpected odometry frames')
        require(abs(od.twist.twist.linear.x)<.015 and abs(od.twist.twist.angular.z)<.03,'Robot not stationary')
        require('final' not in latest or not nonzero(latest['final']), 'Unexpected motion before goal')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        start_xy=(p.x,p.y)
        goal=NavigateToPose.Goal()
        goal.pose.header.frame_id='odom'; goal.pose.header.stamp=od.header.stamp
        goal.pose.pose.position.x=p.x+.4*math.cos(yaw); goal.pose.pose.position.y=p.y+.4*math.sin(yaw)
        goal.pose.pose.orientation=q
        print('PASS: live preflight; sending one 0.40 m goal',flush=True)
        event('goal_sent',start_xy=start_xy)
        sent=client.send_goal_async(goal)
        deadline=time.monotonic()+5
        while not sent.done():
            spin(); live(); require(time.monotonic()<deadline,'Goal acknowledgement timeout')
        handle=sent.result(); require(handle.accepted,'Goal rejected')
        future=handle.get_result_async()
        deadline=time.monotonic()+18
        last_graph=0; last_advance=0; last_clock=None
        drop_sim=None; zero=None; evidence=None
        while time.monotonic()<deadline:
            spin(); live()
            require(not future.done(),'Goal ended before completing fault evidence')
            now=time.monotonic(); sim=clocks[-1]
            if last_clock is None or sim>last_clock:
                last_advance=now
            last_clock=sim
            require(now-last_advance<1.5,'Simulation clock stalled')
            if now-last_graph>.5:
                graph(); last_graph=now
            d=travel(); require(d<.30, f'Travel guard exceeded: {d:.3f} m')
            final=latest.get('final'); up=latest.get('up')
            if final is not None and nonzero(final):
                require(-.005<=final.linear.x<=.125 and abs(final.angular.z)<=.205,'Unexpected final command')
            if not dropped and d>=.025 and final is not None and up is not None and final.linear.x>.025 and up.linear.x>.025 and now-latest['final_wall']<.4 and now-latest['up_wall']<.4:
                require(forwarded is not None and -.2<=sim-forwarded<=.6,'Stale scan before injection')
                dropped=True; drop_sim=sim
                event('drop', last_forwarded_stamp=forwarded, displacement_m=d)
                print(f'SCAN GATE CLOSED at sim={sim:.3f}s, last scan stamp={forwarded:.3f}s; raw scan stays live',flush=True)
            if not dropped:
                require(d<.075, 'Failed to inject fault early'); continue
            age=sim-forwarded
            finals=[e for e in events if e['kind']=='final' and e['sim_s'] is not None and e['sim_s']>=drop_sim]
            zeros=[e for e in finals if not e['nonzero'] and e['sim_s']-forwarded>=1.35]
            if zero is None and zeros:
                zero=zeros[0]
                event('first_zero', sample=zero, source_age_s=zero['sim_s']-forwarded)
                print(f'Observed zero output at estimated source age {zero["sim_s"]-forwarded:.3f}s',flush=True)
            if zero:
                require(not any(e['nonzero'] and e['wall_ns']>zero['wall_ns'] for e in finals),'Nonzero final command after stop')
            if age>2.05:
                require(zero is not None,'No zero output within timeout + 0.55 simulation seconds')
            if zero and sim-zero['sim_s']>=.5:
                pre=[e for e in finals if e['nonzero'] and e['wall_ns']<zero['wall_ns'] and e['sim_s']-forwarded>=.8]
                after=[e for e in events if e['kind']=='upstream' and e['nonzero'] and e['wall_ns']>zero['wall_ns']]
                raw=[e for e in events if e['kind']=='scan' and not e['forwarded'] and e['wall_ns']>zero['wall_ns']]
                if pre and len(after)>=2 and len(raw)>=2 and len([e for e in zeros if e['wall_ns']>=zero['wall_ns']])>=2:
                    evidence=dict(last_forwarded_stamp=forwarded, drop_sim_s=drop_sim, first_zero_sim_s=zero['sim_s'], source_age_at_zero_s=zero['sim_s']-forwarded, upstream_nonzero_after_stop=len(after), raw_scans_after_stop=len(raw), displacement_m=d)
                    break
            require(age<2.8,'Insufficient independent stop evidence')
        require(evidence is not None,'Motion/fault watchdog expired without evidence')
        print('Stop evidence captured; cancelling goal without reopening gate.',flush=True)
        cancelled=handle.cancel_goal_async(); deadline=time.monotonic()+5
        while not (cancelled.done() and future.done()):
            spin(); live(); require(travel()<.30,'Travel guard during cancellation')
            require(time.monotonic()<deadline,'Cancellation/result timeout')
            require(not any(e['kind']=='final' and e['nonzero'] and e['wall_ns']>zero['wall_ns'] for e in events),'Unexpected resumed motion')
        require(future.result().status==GoalStatus.STATUS_CANCELED,'Goal did not report CANCELED')
        evidence['action_status']='CANCELED'
        (directory/'candidate.json').write_text(json.dumps(evidence,indent=2)+'\n')
        print(json.dumps(evidence,indent=2),flush=True)
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        print(f'DRIVER ABORT: {type(exc).__name__}: {exc}',flush=True)
        # Never reopen the gate. Parent requests simulator shutdown immediately.
        dropped=True
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        return 1
    finally:
        log.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def run(config):
    import yaml
    import xml.etree.ElementTree as ET
    require(os.environ.get('ROS_DISTRO')=='jazzy' and os.environ.get('ROS_DOMAIN_ID')=='0' and os.environ.get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp','Source ROS Jazzy and the known environment first')
    base.check_local_processes()
    root=base.ROOT; downloads=base.DOWNLOADS
    original=yaml.safe_load(config.read_text())
    baseline=yaml.safe_load((root/'config/nav2/nav0/echo_nav0_params.yaml').read_text())
    require(all(original[k]==baseline[k] for k in original if k!='bt_navigator'),'Navigation parameters differ from baseline')
    nav=original['bt_navigator']['ros__parameters']; bt=Path(nav['default_nav_to_pose_bt_xml'])
    require(nav['navigators']==['navigate_to_pose'] and bt.is_file(),'Expected no-recovery BT')
    require(not any(x.tag in {'Spin','BackUp','RecoveryNode','DriveOnHeading'} for x in ET.parse(bt).iter()),'Unexpected recovery in BT')
    params=copy.deepcopy(original); cm=params['collision_monitor']['ros__parameters']
    require(cm['scan']['topic']=='/scan' and cm['scan']['source_timeout']==TIMEOUT and cm['source_timeout']==TIMEOUT and cm['observation_sources']==['scan'],'Unexpected monitor configuration')
    cm['scan']['topic']=GATE
    env=dict(os.environ,ECHO_S03_DEPTH_CY_MODE='baseline',PYTHONUNBUFFERED='1'); env.pop('ECHO_S04_OBSTACLE_DIR',None)
    sim=[sys.executable,'-u',str(root/'scripts/start_echo_dynamic_four_d455.py'),'--calibration','coupled_xy','--agents','none']
    check=subprocess.run(sim+['--check'],cwd=root,env=env,capture_output=True,text=True,timeout=20)
    print(check.stdout,end='',flush=True)
    require(check.returncode==0 and 'Existing experiment processes: 0' in check.stdout,'Supervisor preflight failed')
    d=Path(tempfile.mkdtemp(prefix='echo_s04_1c_stale_'+time.strftime('%Y%m%d_%H%M%S')+'_',dir=downloads))
    shutil.copy2(config,d/'baseline_params.yaml'); shutil.copy2(bt,d/'input_behavior_tree.xml')
    (d/'trial_params.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    (d/'repo_state.txt').write_text(subprocess.check_output(['git','status','--short','--branch'],cwd=root,text=True)+subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True))
    shutil.copy2(Path(__file__),d/'runner.py')
    print(f'TRIAL: {d}',flush=True)
    procs={}; outcome='ABORT: not completed'; motion=False
    def start(name,cmd):
        with (d/(name+'.log')).open('x') as out:
            procs[name]=subprocess.Popen(cmd,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
    def alive(names):
        for name in names: require(procs[name].poll() is None,f'{name} exited')
    def wait_text(name,txt,sec,names):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            alive(names)
            if txt in base.read_log(d/(name+'.log')): return
            time.sleep(.15)
        raise RuntimeError(f'Timeout waiting for {name}: {txt}')
    def send(name,sig):
        p=procs.get(name)
        if p is not None and p.poll() is None:
            try: os.killpg(p.pid,sig)
            except ProcessLookupError: pass
    def stop(name,seconds):
        p=procs.get(name)
        if p is None or p.poll() is not None: return
        send(name,signal.SIGINT)
        try: p.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            send(name,signal.SIGTERM)
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired: print(f'WARNING: {name} PID {p.pid} still running; inspect manually',flush=True)
    try:
        start('supervisor',sim)
        print('Starting simulator. Leave Isaac Sim NOT PLAYING.',flush=True)
        wait_text('supervisor','Press Play in Isaac Sim',210,['supervisor'])
        start('driver',[sys.executable,'-u',str(Path(__file__).resolve()),'--driver',str(d)])
        wait_text('driver','GATE READY',20,['supervisor','driver'])
        start('nav0',['ros2','launch',str(root/'launch/echo_nav0.launch.py'),f'params_file:={d/"trial_params.yaml"}'])
        start('recorder',['ros2','bag','record','-s','mcap','-o',str(d/'bag'),'--disable-keyboard-controls','--topics',*TOPICS])
        wait_text('recorder','Recording...',25,['supervisor','driver','nav0','recorder'])
        print('READY: inspect the robot and confirm at least 1 m of clear forward space. No box should be present. Press Play ONCE, then return here.',flush=True)
        wait_text('nav0','Managed nodes are active',120,['supervisor','driver','nav0','recorder'])
        wait_text('recorder','All requested topics are subscribed',20,['supervisor','driver','nav0','recorder'])
        print('GO authorizes one 0.40 m forward goal and one scan interruption. Watch the robot; pause immediately if unsafe. Authorization (20 seconds): ',end='',flush=True)
        require(bool(select.select([sys.stdin],[],[],20)[0]),'Authorization timeout')
        require(sys.stdin.readline().strip()=='GO','Operator declined')
        motion=True; (d/'authorize').touch(); deadline=time.monotonic()+40; printed=0
        while procs['driver'].poll() is None:
            alive(['supervisor','nav0','recorder'])
            s=base.read_log(d/'driver.log'); print(s[printed:],end='',flush=True); printed=len(s)
            require(time.monotonic()<deadline,'Outer test watchdog expired'); time.sleep(.1)
        print(base.read_log(d/'driver.log')[printed:],end='',flush=True)
        require(procs['driver'].returncode==0 and (d/'candidate.json').is_file(),'Driver failed; inspect evidence')
        navlog=base.read_log(d/'nav0.log')
        require('Robot to stop due to invalid source' in navlog and 'Latest source and current collision monitor node timestamps differ' in navlog,'No matching stale-source diagnostic; evidence requires review')
        outcome='CANDIDATE PASS: stale-input stop, upstream commands blocked, cancellation confirmed; verify bag'
    except (Exception,KeyboardInterrupt) as e:
        outcome=f'ABORT: {type(e).__name__}: {e}'
        print(outcome,flush=True)
        if motion: print('If robot is moving, pause Isaac Sim immediately.',flush=True)
    finally:
        # Stop simulator first, never restore the gate or publish direct motor commands.
        send('supervisor',signal.SIGINT)
        stop('driver',5); stop('supervisor',20); stop('recorder',15); stop('nav0',12)
        logs=re.findall(r'\[ECHO\] Logs: (\S+)',base.read_log(d/'supervisor.log'))
        if logs and Path(logs[-1]).is_dir():
            dest=d/'supervisor_child_logs'; dest.mkdir(exist_ok=True)
            for p in Path(logs[-1]).glob('*.log'): shutil.copy2(p,dest/p.name)
        (d/'outcome.txt').write_text(outcome+'\n')
        if (d/'bag/metadata.yaml').is_file():
            info=subprocess.run(['ros2','bag','info',str(d/'bag')],capture_output=True,text=True,timeout=20)
            (d/'bag_info.txt').write_text(info.stdout+info.stderr); print(info.stdout,flush=True)
        archive=shutil.make_archive(str(d),'zip',root_dir=d.parent,base_dir=d.name)
        print(f'OUTCOME: {outcome}\nEVIDENCE: {d}\nUPLOAD: {archive}',flush=True)
    return 0 if outcome.startswith('CANDIDATE PASS') else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--config',type=Path,default=base.DEFAULT_CONFIG); p.add_argument('--driver',type=Path,help=argparse.SUPPRESS); a=p.parse_args()
    try: sys.exit(driver(a.driver) if a.driver else run(a.config))
    except (Exception,KeyboardInterrupt) as e:
        print(f'FAIL: {type(e).__name__}: {e}',file=sys.stderr); sys.exit(1)
