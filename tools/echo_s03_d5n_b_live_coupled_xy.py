#!/usr/bin/env python3
"""D5N-B: temporary live four-camera coupled-XY RGBD pipeline and A/B audit.
Uses existing images/TF; changes only cloned CameraInfo, on separate topics.
No cmd_vel, SLAM, file edits or production publications. ROS Jazzy required.
"""
import argparse
from collections import OrderedDict, Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import os
import signal
import subprocess
import time
import zipfile
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from rtabmap_msgs.msg import RGBDImages

SIDES = ('front', 'rear', 'left', 'right')
BASE = '/echo/cameras'
PROD = '/echo/s03/rgbd_images'
FX = 317.04311998


def stamp(msg):
    s = msg.header.stamp
    return int(s.sec) * 1_000_000_000 + int(s.nanosec)


def model_ok(info, xy):
    cx, cy = xy
    return (info.width, info.height) == (640, 480) and np.allclose(
        [info.k[0], info.k[4], info.k[2], info.k[5], info.p[0], info.p[5], info.p[2], info.p[6]],
        [FX, FX, cx, cy, FX, FX, cx, cy], rtol=0, atol=1e-4)


def depth_array(im):
    if im.encoding != '32FC1' or (im.width, im.height) != (640, 480):
        raise ValueError(f'Unexpected registered depth: {im.encoding} {im.width}x{im.height}')
    return np.ndarray((im.height, im.width), dtype=('>f4' if im.is_bigendian else '<f4'),
                      buffer=im.data, strides=(im.step, 4)).copy()


def compare_depth(a, b):
    va = np.isfinite(a) & (a > 0)
    vb = np.isfinite(b) & (b > 0)
    both = va & vb
    n = int(both.sum())
    diff = np.abs(a[both].astype(np.float64) - b[both])
    return dict(valid_baseline=int(va.sum()), valid_candidate=int(vb.sum()),
                mask_disagreements=int((va != vb).sum()), common_valid=n,
                exact_equal=int(np.count_nonzero(diff == 0)),
                max_abs_difference_m=float(diff.max()) if n else None,
                mean_abs_difference_m=float(diff.mean()) if n else None)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=25, help='wall-clock comparison window after warmup')
    p.add_argument('--warmup', type=float, default=20)
    p.add_argument('--output-dir', type=Path, default=Path.home() / 'Downloads')
    args = p.parse_args()
    if not (5 <= args.seconds <= 120 and 5 <= args.warmup <= 60):
        p.error('seconds must be 5..120; warmup 5..60')
    tag = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    ns = '/echo/s03/d5n_b_' + str(os.getpid())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    work = args.output_dir / ('echo_s03_d5n_b_logs_' + tag)
    work.mkdir()
    out = args.output_dir / ('echo_s03_d5n_b_live_' + tag + '.zip')
    rclpy.init()
    node = Node('echo_s03_d5n_b_audit_' + str(os.getpid()))
    pubs, subs, procs, handles = [], [], [], []
    modes = ('baseline', 'candidate')
    relay_counts = Counter(); relay_errors = Counter(); message_errors = Counter()
    aggregates = {m: [] for m in modes}
    pending = {(m, s): OrderedDict() for m in modes for s in SIDES}
    seen = {s: set() for s in SIDES}; pairs = {s: [] for s in SIDES}
    phase = 'warmup'; ready = set()
    report = dict(study='S03-D5N-B live candidate transport', namespace=ns,
                  status='incomplete', baseline_topic=PROD, candidate_topic=ns+'/rgbd_images',
                  notes=['Uses unchanged live images and existing TF. No ground-truth or SLAM accuracy claim.',
                         'Parallel pipeline adds compute load; measured rates are not standalone performance.',
                         'Pairs use exact embedded RGB and registered-depth timestamps and identical RGB payloads.'])

    def spawn(name, pkg, executable, remaps, params):
        path = work / (name + '.log')
        f = path.open('w'); handles.append(f)
        cmd = ['ros2', 'run', pkg, executable, '--ros-args', '-r', '__ns:='+ns, '-r', '__node:='+name]
        for k, v in params.items():
            cmd += ['-p', f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
        for k, v in remaps.items(): cmd += ['-r', k+':='+v]
        print('START', name, flush=True)
        procs.append((name, subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)))

    def on_info(msg, key, pub):
        side, sensor = key
        if not model_ok(msg, (320., 240.)) or msg.header.frame_id != f'echo_d455_{side}_{sensor}_optical_frame':
            relay_errors['/'.join(key)] += 1
            return
        ci = deepcopy(msg)
        ci.k[2] = ci.p[2] = 319.5
        ci.k[5] = ci.p[6] = 239.5
        pub.publish(ci)
        relay_counts['/'.join(key)] += 1

    def on_aggregate(msg, mode):
        try:
            items = msg.rgbd_images
            if len(items) != 4:
                raise ValueError(f'{len(items)} cameras, expected 4')
            xy = (320., 240.) if mode == 'baseline' else (319.5, 239.5)
            times = []
            parsed = []
            for side, item in zip(SIDES, items):
                expected = f'echo_d455_{side}_color_optical_frame'
                # Registered depth must be expressed in the RGB optical frame.
                for name, info in [('rgb',item.rgb_camera_info), ('depth',item.depth_camera_info)]:
                    if not model_ok(info, xy) or info.header.frame_id != expected:
                        raise ValueError(f'{side} {name} info model/frame mismatch: {info.header.frame_id} K={list(info.k)}')
                if item.rgb.header.frame_id != expected or item.depth.header.frame_id != expected:
                    raise ValueError(f'{side} image frame mismatch')
                rgb_t, dep_t = stamp(item.rgb), stamp(item.depth)
                if rgb_t != dep_t: raise ValueError(f'{side} RGB/depth stamp mismatch {rgb_t-dep_t} ns')
                times.append(rgb_t)
                parsed.append((side, item, (rgb_t,dep_t)))
            ready.add(mode)
            if phase != 'measure': return
            aggregates[mode].append(dict(wall_s=time.monotonic(), stamps_ns=times,
                                         spread_ms=(max(times)-min(times))/1e6))
            other = 'candidate' if mode == 'baseline' else 'baseline'
            for side,item,key in parsed:
                if key in seen[side]: continue
                entry = (hashlib.sha256(bytes(item.rgb.data)).hexdigest(), depth_array(item.depth))
                opposite = pending[other,side].pop(key,None)
                if opposite is None:
                    cache = pending[mode,side]; cache[key] = entry
                    while len(cache)>12: cache.popitem(last=False)
                    continue
                seen[side].add(key)
                if entry[0] != opposite[0]:
                    message_errors[f'{side} matched stamp different RGB payload'] += 1
                    continue
                a,b = (entry[1],opposite[1]) if mode=='baseline' else (opposite[1],entry[1])
                metrics = compare_depth(a,b); metrics['stamp_ns'] = key[0]; pairs[side].append(metrics)
        except Exception as exc:
            key = mode+': '+str(exc)
            message_errors[key] += 1
            if message_errors[key] == 1: print('AUDIT:', key, flush=True)

    def spin_until(deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            for name,proc in procs:
                if proc.poll() is not None: raise RuntimeError(f'{name} exited with {proc.returncode}')
            rclpy.spin_once(node, timeout_sec=.1)

    try:
        print('ISOLATED TOPIC ROOT:', ns, flush=True)
        # Subscribe before launching; no writes to original sensor topics.
        for side in SIDES:
            src = BASE+'/'+side; dst = ns+'/'+side
            for sensor in ('color','depth'):
                pub = node.create_publisher(CameraInfo, dst+'/'+sensor+'/camera_info', 10)
                pubs.append(pub)
                subs.append(node.create_subscription(CameraInfo, src+'/'+sensor+'/camera_info',
                    lambda msg,key=(side,sensor),pub=pub:on_info(msg,key,pub), qos_profile_sensor_data))
            spawn(side+'_register','depth_image_proc','register_node',{
                'depth/image_rect':src+'/depth/image_raw', 'depth/camera_info':dst+'/depth/camera_info',
                'rgb/camera_info':dst+'/color/camera_info',
                'depth_registered/image_rect':dst+'/depth_registered/image_rect',
                'depth_registered/camera_info':dst+'/depth_registered/camera_info'}, {'use_sim_time':True})
            spawn(side+'_rgbd_sync','rtabmap_sync','rgbd_sync',{
                'rgb/image':src+'/color/image_raw','rgb/camera_info':dst+'/color/camera_info',
                'depth/image':dst+'/depth_registered/image_rect','rgbd_image':dst+'/rgbd_image'},
                {'use_sim_time':True,'approx_sync':False,'qos':2,'topic_queue_size':20,'sync_queue_size':20})
        spawn('rgbdx_sync','rtabmap_sync','rgbdx_sync',
              {**{f'rgbd_image{i}':ns+'/'+s+'/rgbd_image' for i,s in enumerate(SIDES)}, 'rgbd_images':ns+'/rgbd_images'},
              {'use_sim_time':True,'rgbd_cameras':4,'approx_sync':True,'approx_sync_max_interval':0.05,
               'qos':2,'topic_queue_size':20,'sync_queue_size':20})
        for mode,topic in [('baseline',PROD),('candidate',ns+'/rgbd_images')]:
            subs.append(node.create_subscription(RGBDImages,topic,lambda msg,m=mode:on_aggregate(msg,m),qos_profile_sensor_data))
        end = time.monotonic()+args.warmup
        while time.monotonic()<end and rclpy.ok() and ready != set(modes):
            spin_until(min(end,time.monotonic()+.5))
        print('Ready streams:', sorted(ready), flush=True)
        phase = 'measure'; begin = time.monotonic(); spin_until(begin+args.seconds); elapsed = time.monotonic()-begin
        report['measurement_wall_seconds'] = elapsed
        report['status'] = 'complete' if ready==set(modes) and all(pairs[s] for s in SIDES) else 'incomplete: missing comparable streams or pairs'
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=repr(exc); print('INCOMPLETE:',repr(exc),flush=True)
    finally:
        for _,proc in procs:
            if proc.poll() is None:
                try: os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError: pass
        for _,proc in procs:
            try: proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try: os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError: pass
                proc.wait()
        for f in handles:f.close()
        report['relay_counts']=dict(relay_counts);report['relay_errors']=dict(relay_errors);report['message_errors']=dict(message_errors)
        report['aggregate_summary']={}
        for mode,rows in aggregates.items():
            dur=rows[-1]['wall_s']-rows[0]['wall_s'] if len(rows)>1 else 0
            report['aggregate_summary'][mode]=dict(count=len(rows),arrival_hz=(len(rows)-1)/dur if dur>0 else None,
                max_spread_ms=max((x['spread_ms'] for x in rows),default=None),
                unique_camera_stamp_sets=len(set(tuple(x['stamps_ns']) for x in rows)))
        report['matched_pairs']={}
        for side,rows in pairs.items():
            n=sum(x['common_valid'] for x in rows)
            report['matched_pairs'][side]=dict(count=len(rows),common_valid=n,
                mask_disagreements=sum(x['mask_disagreements'] for x in rows),
                exact_fraction=sum(x['exact_equal'] for x in rows)/n if n else None,
                max_abs_difference_m=max((x['max_abs_difference_m'] for x in rows if x['max_abs_difference_m'] is not None),default=None))
        print(json.dumps({k:report[k] for k in ('status','relay_errors','message_errors','aggregate_summary','matched_pairs')},indent=2),flush=True)
        with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as z:
            z.writestr('report.json',json.dumps(report,indent=2,allow_nan=False))
            z.writestr('aggregate_samples.json',json.dumps(aggregates,indent=2))
            z.writestr('pair_metrics.json',json.dumps(pairs,indent=2,allow_nan=False))
            z.write(Path(__file__),'source.py')
            for f in work.glob('*.log'):z.write(f,'logs/'+f.name)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print('RESULT ZIP:',out,flush=True)
    return 0 if report['status']=='complete' else 2


if __name__=='__main__':raise SystemExit(main())
