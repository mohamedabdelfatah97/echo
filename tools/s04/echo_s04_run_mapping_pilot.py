#!/usr/bin/env python3
"""One-terminal supervised static S04 route + common-input capture.
Requires echo_s04_run_obstacle_v2.py in the same directory and pose logger patch.
Raw MCAP stays local; only small evidence ZIP is created. No direct cmd_vel.
"""
import argparse, json, math, os, re, select, shutil, signal, subprocess, sys, tempfile, time
from pathlib import Path
from collections import deque
import echo_s04_run_obstacle_v2 as base

RGBD='/echo/s03/rgbd_images'
LIDAR='/front_3d_lidar/lidar_points'
TOPICS=base.TOPICS+[RGBD,LIDAR]
# In starting odom-relative coordinates (m, m, degrees). Small zigzag pilot.
ROUTE=[(.60,0.,0.),(1.10,.35,35.),(1.60,0.,-35.)]
GIB=1024**3

def require(ok,msg):
    if not ok: raise RuntimeError(msg)

def stamp(msg): return msg.sec+msg.nanosec*1e-9

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return math.atan2(math.sin(a),math.cos(a))

def driver(d):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan,PointCloud2
    from rtabmap_msgs.msg import RGBDImages
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer,TransformListener
    rclpy.init(args=[]); n=Node('echo_s04_mapping_pilot'); buf=Buffer(); listener=TransformListener(buf,n)
    client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
    latest={}; counts={}; clocks=deque(maxlen=50); backwards=False; handle=None; result=None; rows=[]
    log=(d/'route_events.jsonl').open('x',buffering=1)
    def event(kind,**kwargs):
        log.write(json.dumps(dict(kind=kind,wall_ns=time.time_ns(),sim_s=clocks[-1] if clocks else None,**kwargs))+'\n')
    def on_clock(m):
        nonlocal backwards
        t=stamp(m.clock)
        if clocks and t<clocks[-1]-1e-6: backwards=True
        clocks.append(t); latest['clock']=(time.monotonic(),t)
    def recv(key,m):
        latest[key]=(time.monotonic(),m); counts[key]=counts.get(key,0)+1
    def rgbd(m):
        # Do not keep giant images in the driver's history.
        latest['rgbd']=(time.monotonic(),(stamp(m.header.stamp),len(m.rgbd_images))); counts['rgbd']=counts.get('rgbd',0)+1
    def lidar(m):
        latest['lidar']=(time.monotonic(),(stamp(m.header.stamp),m.width*m.height)); counts['lidar']=counts.get('lidar',0)+1
    def final(m): recv('final',m)
    for typ,topic,cb in [(Clock,'/clock',on_clock),(Odometry,'/chassis/odom',lambda m:recv('odom',m)),(LaserScan,'/scan',lambda m:recv('scan',m)),(RGBDImages,RGBD,rgbd),(PointCloud2,LIDAR,lidar)]:
        n.create_subscription(typ,topic,cb,QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT))
    n.create_subscription(Twist,'/cmd_vel',final,20)
    services={name:n.create_client(GetState,f'/{name}/get_state') for name in ('controller_server','collision_monitor','bt_navigator')}
    def spin(): rclpy.spin_once(n,timeout_sec=.03)
    def graph():
        pubs=lambda topic: sorted((p.node_namespace,p.node_name) for p in n.get_publishers_info_by_topic(topic))
        require(pubs('/cmd_vel')==[('/','collision_monitor')],'Unexpected final command publishers')
        require(pubs('/cmd_vel_smoothed')==[('/','velocity_smoother')],'Unexpected smoother publishers')
        require(pubs('/scan')==[('/','echo_nav0_pointcloud_to_laserscan')],'Unexpected scan publishers')
        require(any(s.node_name=='collision_monitor' for s in n.get_subscriptions_info_by_topic('/scan')),'Monitor not subscribed to real scan')
    def live():
        require(not backwards,'Simulation clock reset')
        now=time.monotonic()
        for k in ('clock','odom','scan','rgbd','lidar'): require(k in latest and now-latest[k][0]<3.,f'Stale {k}')
        sim=clocks[-1]; scan=latest['scan'][1]; od=latest['odom'][1]
        require(od.header.frame_id=='odom' and od.child_frame_id=='base_link','Unexpected odom frames')
        for k,t in [('scan',stamp(scan.header.stamp)),('odom',stamp(od.header.stamp)),('rgbd',latest['rgbd'][1][0]),('lidar',latest['lidar'][1][0])]: require(-.25<=sim-t<=.75,f'{k} stamp out of range: {sim-t:.3f}')
        require(latest['rgbd'][1][1]==4 and latest['lidar'][1][1]>0,'Invalid aggregate or cloud')
        gt=base.latest_gt(d); require(0<=(time.time_ns()-gt['wall_utc_ns'])/1e9<=2. and abs(sim-gt['sim_time_s'])<=.75,'Ground truth not fresh')
        if 'final' in latest:
            v=latest['final'][1]; require(-.005<=v.linear.x<=.125 and abs(v.linear.y)<.005 and abs(v.angular.z)<=.205,'Unexpected velocity')
        return od,gt
    def pose():
        p=latest['odom'][1].pose.pose; return p.position.x,p.position.y,yaw(p.orientation)
    try:
        print('DRIVER READY',flush=True)
        end=time.monotonic()+240
        while not (d/'authorize').exists(): require(time.monotonic()<end,'Authorization timeout'); spin()
        end=time.monotonic()+15; futures={}
        while time.monotonic()<end:
            spin()
            for k,c in services.items():
                if k not in futures and c.service_is_ready(): futures[k]=c.call_async(GetState.Request())
            if len(clocks)>2 and clocks[-1]>clocks[0] and all(counts.get(k,0)>=2 for k in ('odom','scan','rgbd','lidar')) and len(futures)==3 and all(f.done() for f in futures.values()) and client.server_is_ready() and buf.can_transform('base_link',latest['scan'][1].header.frame_id,Time()): break
        else: raise RuntimeError('Incomplete startup')
        for k,f in futures.items(): require(f.result().current_state.id==3,f'{k} not active')
        graph(); od,gt=live(); start=pose(); gx0,gy0=gt['xyz_m'][:2]; syaw=start[2]
        require(abs(od.twist.twist.linear.x)<.015 and abs(od.twist.twist.angular.z)<.03,'Not stationary at start')
        tf=buf.lookup_transform('base_link',latest['scan'][1].header.frame_id,Time()).transform
        t,q=tf.translation,tf.rotation
        require(abs(t.x+.232)<.02 and abs(t.y)<.02 and max(abs(q.x),abs(q.y),abs(q.z))<.01,'Unexpected scan transform')
        scan=latest['scan'][1]; zone=0
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                a=scan.angle_min+i*scan.angle_increment; x=r*math.cos(a)+t.x; y=r*math.sin(a)+t.y
                zone+=(-.82<=x<=.40 and abs(y)<=.40)
        require(zone<4,'Initial StopZone occupied')
        event('route_start',odom=start,gt=gt['xyz_m'],route=ROUTE)
        print('PASS: camera, LiDAR, TF, odom, ground truth and navigation preflight',flush=True)
        c,s=math.cos(syaw),math.sin(syaw)
        lastgraph=0.; headings=[]
        for i,(x,y,deg) in enumerate(ROUTE,1):
            od,gt=live(); graph(); target=(start[0]+c*x-s*y,start[1]+s*x+c*y,wrap(syaw+math.radians(deg)))
            goal=NavigateToPose.Goal(); goal.pose.header.frame_id='odom'; goal.pose.header.stamp=od.header.stamp
            goal.pose.pose.position.x,goal.pose.pose.position.y=target[:2]; goal.pose.pose.orientation.z=math.sin(target[2]/2); goal.pose.pose.orientation.w=math.cos(target[2]/2)
            event('goal_sent',index=i,target=target,odom=pose()); print(f'GOAL {i}/3: relative ({x:.2f}, {y:.2f}) m, yaw {deg:+.0f} deg',flush=True)
            future=client.send_goal_async(goal); end=time.monotonic()+6
            while not future.done(): spin(); live(); require(time.monotonic()<end,'Goal acknowledgement timeout')
            handle=future.result(); require(handle.accepted,'Goal rejected'); result=handle.get_result_async(); end=time.monotonic()+60
            while not result.done():
                spin(); od,gt=live(); px,py,pa=pose(); now=time.monotonic()
                require(math.hypot(gt['xyz_m'][0]-gx0,gt['xyz_m'][1]-gy0)<2.2,'GT travel envelope exceeded')
                dx,dy=px-start[0],py-start[1]; lx,ly=c*dx+s*dy,-s*dx+c*dy
                require(-.2<=lx<=1.95 and -.5<=ly<=.85,'Route envelope exceeded')
                require(now<end,'Goal watchdog expired')
                if now-lastgraph>1: graph(); lastgraph=now
            status=result.result().status; handle=None
            require(status==GoalStatus.STATUS_SUCCEEDED,f'Goal {i} status {status}')
            px,py,pa=pose(); poserr=math.hypot(px-target[0],py-target[1]); yawerr=abs(wrap(pa-target[2])); require(poserr<=.15 and yawerr<=.27,'Goal terminal pose outside pilot bounds')
            row=dict(index=i,status='SUCCEEDED',target=target,actual=(px,py,pa),position_error_m=poserr,yaw_error_deg=math.degrees(yawerr)); rows.append(row); headings.append(pa); event('goal_result',**row); print(json.dumps(row),flush=True)
        require(abs(wrap(headings[-1]-headings[-2]))>=math.radians(35),'Insufficient heading change')
        end=time.monotonic()+3
        while time.monotonic()<end: spin(); live()
        event('route_end',odom=pose(),counts=counts)
        (d/'candidate.json').write_text(json.dumps(dict(goals=rows,counts=counts,sim_end=clocks[-1]),indent=2)+'\n')
        return 0
    except (Exception,KeyboardInterrupt) as e:
        print(f'DRIVER ABORT: {type(e).__name__}: {e}',flush=True)
        if handle is not None:
            try:
                f=handle.cancel_goal_async(); end=time.monotonic()+1
                while not f.done() and time.monotonic()<end: spin()
            except Exception: pass
        return 1
    finally:
        log.close(); n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def run(config):
    import yaml
    import xml.etree.ElementTree as ET
    require(os.environ.get('ROS_DISTRO')=='jazzy' and os.environ.get('ROS_DOMAIN_ID')=='0' and os.environ.get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp','Source known Jazzy environment')
    root=base.ROOT; downloads=base.DOWNLOADS; base.check_local_processes()
    require((root/'tools/s04/pose_logger.py').is_file() and 'ECHO_S04_POSE_DIR' in (root/'scripts/start_full_warehouse_two_humans_four_d455.py').read_text(),'Apply pose logger patch first')
    require(shutil.disk_usage(downloads).free>=40*GIB,'Need at least 40 GiB free in Downloads')
    params=yaml.safe_load(config.read_text()); baseline=yaml.safe_load((root/'config/nav2/nav0/echo_nav0_params.yaml').read_text()); require(all(params[k]==baseline[k] for k in baseline if k!='bt_navigator'),'NAV-0 config differs from original')
    nav=params['bt_navigator']['ros__parameters']; bt=Path(nav['default_nav_to_pose_bt_xml']); require(nav['navigators']==['navigate_to_pose'] and bt.is_file(),'Missing no-recovery BT'); require(not any(x.tag in {'Spin','BackUp','RecoveryNode','DriveOnHeading'} for x in ET.parse(bt).iter()),'Unexpected recovery in BT')
    env=dict(os.environ,ECHO_S03_DEPTH_CY_MODE='baseline',PYTHONUNBUFFERED='1'); env.pop('ECHO_S04_OBSTACLE_DIR',None); env.pop('ECHO_S04_POSE_DIR',None)
    sim=[sys.executable,'-u',str(root/'scripts/start_echo_dynamic_four_d455.py'),'--calibration','coupled_xy','--agents','none']
    check=subprocess.run(sim+['--check'],cwd=root,env=env,capture_output=True,text=True,timeout=20); print(check.stdout,end='',flush=True); require(check.returncode==0 and 'Existing experiment processes: 0' in check.stdout,'Supervisor preflight failed')
    d=Path(tempfile.mkdtemp(prefix='echo_s04_2_mapping_pilot_'+time.strftime('%Y%m%d_%H%M%S')+'_',dir=downloads)); env['ECHO_S04_POSE_DIR']=str(d)
    shutil.copy2(config,d/'nav0_params.yaml'); shutil.copy2(bt,d/'behavior_tree.xml'); shutil.copy2(Path(__file__),d/'runner.py')
    (d/'repo_state.txt').write_text(subprocess.check_output(['git','status','--short','--branch'],cwd=root,text=True)+subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True)); (d/'route.json').write_text(json.dumps(ROUTE)+'\n')
    procs={}; outcome='ABORT: not completed'; print('TRIAL:',d,flush=True)
    def start(name,cmd):
        with (d/(name+'.log')).open('x') as out: procs[name]=subprocess.Popen(cmd,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
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
            except subprocess.TimeoutExpired: print(f'WARNING: {name} PID {p.pid} still running',flush=True)
    try:
        start('supervisor',sim); print('Starting simulator. Leave Isaac Sim NOT PLAYING.',flush=True); wait_text('supervisor','Press Play in Isaac Sim',210,['supervisor']); require((d/'physics.jsonl').is_file(),'No passive pose logger')
        start('driver',[sys.executable,'-u',str(Path(__file__).resolve()),'--driver',str(d)]); wait_text('driver','DRIVER READY',20,['driver','supervisor'])
        start('nav0',['ros2','launch',str(root/'launch/echo_nav0.launch.py'),f'params_file:={d/"nav0_params.yaml"}'])
        start('recorder',['ros2','bag','record','-s','mcap','-o',str(d/'bag'),'--disable-keyboard-controls','--max-bag-size',str(2*GIB),'--topics',*TOPICS]); wait_text('recorder','Recording...',25,['supervisor','driver','nav0','recorder'])
        print('READY: no box/agents. Inspect at least 3 m clear forward and 1.5 m clear on both sides and behind; then press Play ONCE. Do not pause except for safety.',flush=True)
        wait_text('nav0','Managed nodes are active',120,['supervisor','driver','nav0','recorder']); wait_text('recorder','All requested topics are subscribed',30,['supervisor','driver','nav0','recorder'])
        print('GO authorizes the three-goal zigzag route. Watch the whole swept area. Authorization (20 seconds): ',end='',flush=True); require(bool(select.select([sys.stdin],[],[],20)[0]),'Authorization timeout'); require(sys.stdin.readline().strip()=='GO','Operator declined'); (d/'authorize').touch()
        end=time.monotonic()+195; printed=0
        while procs['driver'].poll() is None:
            alive(['supervisor','nav0','recorder']); text=base.read_log(d/'driver.log'); print(text[printed:],end='',flush=True); printed=len(text)
            require(time.monotonic()<end,'Outer watchdog expired'); require(shutil.disk_usage(downloads).free>=15*GIB,'Free disk guard'); require(sum(p.stat().st_size for p in (d/'bag').glob('*.mcap'))<12*GIB,'Pilot recording size guard'); time.sleep(.15)
        print(base.read_log(d/'driver.log')[printed:],end='',flush=True); require(procs['driver'].returncode==0 and (d/'candidate.json').is_file(),'Driver failed; inspect evidence'); outcome='CANDIDATE PASS: route and sensor capture completed; inspect MCAP before comparison'
    except (Exception,KeyboardInterrupt) as e:
        outcome=f'ABORT: {type(e).__name__}: {e}'; print(outcome,'\nPause simulator immediately if moving.',flush=True)
    finally:
        send('supervisor',signal.SIGINT); stop('driver',5); stop('supervisor',20); stop('recorder',30); stop('nav0',12)
        logs=re.findall(r'\[ECHO\] Logs: (\S+)',base.read_log(d/'supervisor.log'))
        if logs and Path(logs[-1]).is_dir():
            dest=d/'supervisor_child_logs'; dest.mkdir(exist_ok=True)
            for p in Path(logs[-1]).glob('*.log'): shutil.copy2(p,dest/p.name)
        (d/'outcome.txt').write_text(outcome+'\n')
        if (d/'bag/metadata.yaml').is_file():
            info=subprocess.run(['ros2','bag','info',str(d/'bag')],capture_output=True,text=True,timeout=30); (d/'bag_info.txt').write_text(info.stdout+info.stderr); print(info.stdout,flush=True)
        # Keep raw recordings untouched, share only small diagnostics.
        import zipfile
        archive=Path(str(d)+'_summary.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(d.rglob('*')):
                if p.is_file() and p.suffix!='.mcap' and p.stat().st_size<20*1024**2: z.write(p,p.relative_to(d.parent))
        print(f'OUTCOME: {outcome}\nRAW DATA: {d}/bag\nUPLOAD: {archive}',flush=True)
    return 0 if outcome.startswith('CANDIDATE PASS') else 1

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--config',type=Path,default=base.DEFAULT_CONFIG); p.add_argument('--driver',type=Path,help=argparse.SUPPRESS); a=p.parse_args()
    try: sys.exit(driver(a.driver) if a.driver else run(a.config))
    except (Exception,KeyboardInterrupt) as e: print(f'FAIL: {type(e).__name__}: {e}',file=sys.stderr); sys.exit(1)
