#!/usr/bin/env python3
"""S04 offline RTAB-Map pipeline pilot; no robot commands, no source bag changes."""
import argparse, collections, json, os, shutil, signal, sqlite3, subprocess, sys, tempfile, time, traceback, zipfile
from pathlib import Path

RGBD='/echo/s03/rgbd_images'; FRONT='/echo/s04/replay/front'; LIDAR='/front_3d_lidar/lidar_points'; ODOM='/chassis/odom'
NS='/echo_s04_replay'; NAME='rtabmap'; MODES=('one_d455','four_d455','lidar')

def require(ok, msg):
    if not ok: raise RuntimeError(msg)

def run_cmd(args, timeout=20):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT, timeout=timeout)

def stop(p):
    if p is None or p.poll() is not None: return
    for sig, timeout in [(signal.SIGINT,30),(signal.SIGTERM,5),(signal.SIGKILL,5)]:
        try: os.killpg(p.pid,sig)
        except ProcessLookupError: return
        try: p.wait(timeout=timeout); return
        except subprocess.TimeoutExpired: pass

def db_summary(path):
    require(path.is_file() and path.stat().st_size>0,'Missing mapper database')
    with sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True) as db:
        integrity=db.execute('PRAGMA integrity_check').fetchone()[0]
        require(integrity=='ok',f'Database integrity: {integrity}')
        tables={x[0] for x in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        require({'Node','Data','Link'}<=tables,'Unexpected RTAB-Map database schema')
        cols={t:[r[1] for r in db.execute(f'PRAGMA table_info({t})')] for t in ('Node','Data','Link')}
        s={'integrity':integrity,'size_bytes':path.stat().st_size,'columns':cols}
        s['nodes']=db.execute('SELECT count(*) FROM Node').fetchone()[0]
        if 'weight' in cols['Node']:
            s['node_weights']=db.execute('SELECT weight,count(*) FROM Node GROUP BY weight').fetchall()
            s['retained_nodes']=db.execute('SELECT count(*) FROM Node WHERE weight>=0').fetchone()[0]
        if 'stamp' in cols['Node']: s['node_stamp_range']=db.execute('SELECT min(stamp),max(stamp) FROM Node').fetchone()
        for k in ('image','depth','scan','calibration'):
            if k in cols['Data']: s[k+'_rows']=db.execute(f'SELECT count(*) FROM Data WHERE length({k})>0').fetchone()[0]
        if 'type' in cols['Link']: s['link_types']=db.execute('SELECT type,count(*) FROM Link GROUP BY type').fetchall()
        return s

def main(args,out,report):
    import yaml
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import RGBDImages, RGBDImage, Info
    trial=args.trial.resolve(strict=True); bag=trial/'bag'; require((bag/'metadata.yaml').is_file(),'Bag metadata not found')
    require(shutil.disk_usage(out).free>20*1024**3,'Need at least 20 GiB free disk space')
    meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']; counts={x['topic_metadata']['name']:x['message_count'] for x in meta['topics_with_message_count']}
    common=['/clock','/tf','/tf_static',ODOM]; require(all(counts.get(t,0)>0 for t in common+[RGBD,LIDAR]),'Required recorded topics absent')
    duration=meta['duration']['nanoseconds']/1e9
    report.update(trial=str(trial),source_counts=counts,source_wall_duration_s=duration,rate=args.rate,domain=args.domain,policy='Replay complete short pilot bag, original recorded /clock and timestamps; no synthesized clock, no actuator topics. RGBD cases share aggregate schedule. LiDAR uses native scan schedule; mapper input acceptance may differ.')
    helptext=run_cmd(['ros2','bag','play','--help']); require(all(x in helptext for x in ('--rate','--topics','--qos-profile-overrides-path','--disable-keyboard-controls')),'Installed rosbag player lacks required CLI options')
    (out/'bag_play_help.txt').write_text(helptext)
    prefix=Path(run_cmd(['ros2','pkg','prefix','rtabmap_slam']).strip()); exe=prefix/'lib/rtabmap_slam/rtabmap'; require(os.access(exe,os.X_OK),f'Missing executable {exe}')
    qosfile=out/'playback_qos.yaml'; qosfile.write_text(yaml.safe_dump({'/tf_static':{'history':'keep_last','depth':100,'reliability':'reliable','durability':'transient_local'}}))
    rclpy.init(args=[]); node=rclpy.create_node('echo_s04_replay_audit')
    def spin_for(s):
        until=time.monotonic()+s
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
    try:
        spin_for(2); others=[(n,ns) for n,ns in node.get_node_names_and_namespaces() if n!=node.get_name()]
        require(not others,f'Replay ROS domain is occupied: {others}')
        report['cases']={}
        for mode in args.modes:
            print(f'\n=== {mode} ===',flush=True)
            d=out/mode; d.mkdir(); db=d/'map.db'; mapper=player=None; subs=[]; pub=None
            stats=collections.Counter(); clock_first=None; clock_last=None; errors=[]; start=time.monotonic(); info_stamps=[]
            def on_clock(msg):
                nonlocal clock_first, clock_last
                t=msg.clock.sec+msg.clock.nanosec/1e9
                if clock_first is None: clock_first=t
                if clock_last is not None and t<clock_last-1e-8: stats['backward_clock']+=1
                clock_last=t; stats['clock']+=1
            def on_info(msg):
                stats['info']+=1
                if getattr(msg,'loop_closure_id',0)>0:stats['loop_closure_reports']+=1
                if getattr(msg,'proximity_detection_id',0)>0:stats['proximity_reports']+=1
            try:
                subs.append(node.create_subscription(Clock,'/clock',on_clock,qos_profile_sensor_data))
                subs.append(node.create_subscription(Info,NS+'/info',on_info,qos_profile_sensor_data))
                is_lidar=mode=='lidar'
                params={'use_sim_time':True,'frame_id':'base_link','publish_tf':False,'subscribe_rgb':False,'subscribe_depth':False,'subscribe_rgbd':not is_lidar,'rgbd_cameras':1 if mode=='one_d455' else 0,'subscribe_scan':False,'subscribe_scan_cloud':is_lidar,'subscribe_odom':True,'subscribe_odom_info':False,'qos':2,'qos_odom':2,'qos_image':2,'qos_camera_info':2,'qos_scan':2,'approx_sync':True,'topic_queue_size':10,'sync_queue_size':10,'database_path':str(db),'Grid/RangeMax':'10.0','Grid/CellSize':'0.05','Grid/DepthDecimation':'4','Grid/3D':'true','Grid/Sensor':'0' if is_lidar else '1','Rtabmap/DetectionRate':'1','RGBD/LinearUpdate':'0.1','RGBD/AngularUpdate':'0.1','Mem/IncrementalMemory':'true','Reg/Strategy':'1' if is_lidar else '0'}
                if is_lidar: params.update({'RGBD/ProximityPathMaxNeighbors':'1'})
                pf=d/'params.yaml'; pf.write_text(yaml.safe_dump({NS+'/'+NAME:{'ros__parameters':params}},sort_keys=False))
                cmd=[str(exe),'--ros-args','-r','__node:='+NAME,'-r','__ns:='+NS,'--params-file',str(pf),'-r','odom:='+ODOM,'-r','rgbd_images:='+RGBD,'-r','rgbd_image:='+FRONT,'-r','scan_cloud:='+LIDAR]
                (d/'command.json').write_text(json.dumps(cmd,indent=2))
                with (d/'mapper.log').open('w') as log:
                    mapper=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
                expected=FRONT if mode=='one_d455' else LIDAR if is_lidar else RGBD
                if mode=='one_d455':
                    pub=node.create_publisher(RGBDImage,FRONT,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
                    def on_aggregate(msg):
                        stats['relay_received']+=1
                        if len(msg.rgbd_images)!=4 or 'front' not in msg.rgbd_images[0].rgb_camera_info.header.frame_id:
                            errors.append('Unexpected camera order/length');return
                        pub.publish(msg.rgbd_images[0]);stats['relay_forwarded']+=1
                    subs.append(node.create_subscription(RGBDImages,RGBD,on_aggregate,qos_profile_sensor_data))
                deadline=time.monotonic()+30
                while time.monotonic()<deadline:
                    require(mapper.poll() is None,'Mapper exited during startup; inspect mapper.log')
                    spin_for(.1)
                    eps=node.get_subscriptions_info_by_topic(expected)
                    if any(x.node_name==NAME and x.node_namespace==NS for x in eps):break
                else: raise RuntimeError(f'Mapper did not subscribe to {expected}')
                spin_for(1)
                topics=common+[LIDAR if is_lidar else RGBD]
                pcmd=['ros2','bag','play',str(bag),'--rate',str(args.rate),'--disable-keyboard-controls','--qos-profile-overrides-path',str(qosfile),'--topics',*topics]
                (d/'playback_command.json').write_text(json.dumps(pcmd,indent=2))
                print('Mapper subscribed; replaying recorded clock and sensor data. No motion topics.',flush=True)
                with (d/'playback.log').open('w') as log:player=subprocess.Popen(pcmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
                deadline=time.monotonic()+duration/args.rate+90; last_print=time.monotonic()
                while player.poll() is None:
                    require(mapper.poll() is None,'Mapper exited during playback')
                    require(time.monotonic()<deadline,'Playback timeout')
                    require(not errors, str(errors))
                    spin_for(.1)
                    if time.monotonic()-last_print>15:
                        print(f'{mode}: clock={clock_last}, info={stats["info"]}, relay={stats["relay_forwarded"]}',flush=True);last_print=time.monotonic()
                require(player.returncode==0,f'Playback exited {player.returncode}')
                spin_for(5); require(not errors,str(errors)); require(stats['clock']>10 and clock_last-clock_first>1 and not stats['backward_clock'],'Recorded clock missing or reset')
                require(stats['info']>0,'No RTAB-Map info received')
                if mode=='one_d455': require(stats['relay_forwarded']>0,'Front relay did not forward data')
                require(mapper.poll() is None,'Mapper exited before requested shutdown')
                stop(mapper); require(mapper.returncode in (0,-signal.SIGINT,130),'Mapper did not exit cleanly')
                s=db_summary(db); require(s['nodes']>1,'Insufficient stored nodes')
                require(s.get('scan_rows' if is_lidar else 'image_rows',0)>0,'Expected stored sensor payload absent')
                if not is_lidar: require(s.get('depth_rows',0)>0,'No stored depth payload')
                s.update(stats=dict(stats),clock_range=[clock_first,clock_last],wall_elapsed_s=time.monotonic()-start,status='CANDIDATE PASS')
                report['cases'][mode]=s; print(json.dumps(s,indent=2),flush=True)
            finally:
                stop(player); stop(mapper)
                for sub in subs:node.destroy_subscription(sub)
                if pub is not None:node.destroy_publisher(pub)
                spin_for(.5)
        report['status']='CANDIDATE PASS: replay pipeline qualification; compare stored observations and maps next'
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);p.add_argument('--rate',type=float,default=.5);p.add_argument('--domain',type=int,default=87);p.add_argument('--modes',nargs='+',choices=MODES,default=list(MODES));a=p.parse_args()
    require(0<a.rate<=1,'Choose replay rate >0 and <=1');require(1<=a.domain<=101,'Choose local replay domain 1..101');require(len(set(a.modes))==len(a.modes),'Duplicate modes')
    os.environ['ROS_DOMAIN_ID']=str(a.domain);os.environ['ROS_LOCALHOST_ONLY']='1';os.environ.pop('ROS_AUTOMATIC_DISCOVERY_RANGE',None);os.environ.pop('ROS_STATIC_PEERS',None)
    out=Path(tempfile.mkdtemp(prefix='echo_s04_rtabmap_pilot_',dir=Path.home()/'Downloads'));report={'status':'FAILED OR INCOMPLETE'};rc=0
    print('OUTPUT:',out,flush=True)
    try:main(a,out,report)
    except (Exception,KeyboardInterrupt) as e:report['error']=repr(e);(out/'error.txt').write_text(traceback.format_exc());print('ABORT:',repr(e),flush=True);rc=1
    finally:
        (out/'report.json').write_text(json.dumps(report,indent=2)); archive=out.with_name(out.name+'_summary.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for f in out.rglob('*'):
                if f.is_file() and f.suffix in ('.txt','.json','.log','.yaml'):z.write(f,f.relative_to(out))
        print('OUTCOME:',report['status']);print('LOCAL DATABASES:',out);print('UPLOAD:',archive);sys.exit(rc)
