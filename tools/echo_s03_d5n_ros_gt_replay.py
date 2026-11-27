#!/usr/bin/env python3
"""D5N-A: fixed-frame ROS consumer replay, isolated from production topics.

Uses four archived native ROS depth frames + concurrent simulator pose trace
(D4F), and archived front renderer sphere data (D5M). Real installed
Jazzy depth_image_proc nodes do the reconstruction. Only D5M uses registration;
its archived local RGB/depth sphere centers supply independent geometry.
No live sensor frames are claimed. Current front static TF is read for register.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import hashlib, io, json, os, signal, subprocess, time, zipfile
import numpy as np

SIDES = ('front', 'rear', 'left', 'right')
MODELS = {'baseline': (320., 240.), 'coupled_xy': (319.5, 239.5)}
TILES = [(-22.,-16.,-5.4,.6),(-16.,-10.,-5.4,.6),(-22.,-16.,-11.4,-5.4),(-16.,-10.,-11.4,-5.4),(-22.,-16.,.6,6.6)]
FLOOR = 2.8610228852699038e-8
D4ZIP = 'echo_s03_d4e_native_ros_depth_20261123T134940Z.zip'
POSES = 'echo_s03_d4f_live_physics_poses_20261123T134930Z.jsonl'
D5ZIP = 'echo_s03_d5m_gt_20261123T233116Z.zip'


def load_inputs(folder):
    with zipfile.ZipFile(folder/D4ZIP) as z:
        meta = json.loads(z.read('capture.json'))
        with np.load(io.BytesIO(z.read('native_depth.npz'))) as f:
            depths = {s:f[s+'_native'].copy() for s in SIDES}
    rows = [json.loads(x) for x in (folder/POSES).read_text().splitlines() if x.strip()]
    if rows[0]['stage_root_layer'].split('/')[-1] != 's01_d455_four_rgbd_raw_trial.usda':
        raise ValueError('Unexpected D4F stage')
    poses = [r for r in rows if r.get('type')=='pose']
    with zipfile.ZipFile(folder/D5ZIP) as z:
        spheres = json.loads(z.read('report.json'))
        with np.load(io.BytesIO(z.read('depth_and_masks.npz'))) as f:
            a = {k:f[k].copy() for k in f.files}
        rgb = z.read('images/rgb_visible.png')
    for s in SIDES:
        d = depths[s]; ci = meta['sides'][s]['camera_info']; im = meta['sides'][s]['image']
        assert d.shape == (480,640) and ci['stamp_s']==im['stamp_s']
        assert ci['K'][2]==320 and ci['K'][5]==240 and ci['P'][2]==320 and ci['P'][6]==240
    return meta,depths,poses,spheres,a,rgb


def stats(x):
    x=np.asarray(x,dtype=float)*1000
    if not len(x): raise ValueError('Empty metric sample')
    return {'n':int(len(x)), 'rmse_mm':float(np.sqrt(np.mean(x*x))), 'mean_mm':float(x.mean()), 'p95_abs_mm':float(np.percentile(abs(x),95))}


def rotation(q):
    w,x,y,z=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def pinhole(d,k):
    v,u=np.indices(d.shape,dtype=float)
    return np.stack((d*(u-k[2])/k[0],d*(v-k[5])/k[4],d),axis=-1)


def world(xyz,pose):
    # ROS optical -> USD camera local (+X right,+Y up,-Z forward).
    return (xyz*np.array([1.,-1.,-1.]))@rotation(pose['quat_wxyz']).T+np.asarray(pose['xyz_m'])


def floor_scores(side,d,clouds,meta,poses):
    t=meta['sides'][side]['image']['stamp_s']; result={}
    for domain in ('simulation_manager_time_s','timeline_time_s'):
        row=min(poses,key=lambda r:abs(r[domain]-t)); dt=row[domain]-t
        if abs(dt)>.0084: raise ValueError(f'{side} {domain}: no pose within 8.4 ms')
        w={m:world(p,row['cameras'][side]) for m,p in clouds.items()}; b=w['baseline']
        # Same D4F baseline-selected geometric candidate mask; not semantic labels.
        xy=np.zeros(d.shape,bool)
        for xmin,xmax,ymin,ymax in TILES:
            xy|=(b[:,:,0]>xmin+.02)&(b[:,:,0]<xmax-.02)&(b[:,:,1]>ymin+.02)&(b[:,:,1]<ymax-.02)
        mask=np.isfinite(d)&(d>=.25)&(d<=8)&xy&(abs(b[:,:,2]-FLOOR)<.06)
        mask &= np.logical_and.reduce([np.isfinite(p).all(axis=2) for p in w.values()])
        if mask.sum()<100: raise ValueError('Insufficient floor samples')
        result[domain]={'pose_dt_s':dt,'models':{m:stats(p[:,:,2][mask]-FLOOR) for m,p in w.items()}}
    return result


def erode(m):
    p=np.pad(m,1,constant_values=False); h,w=m.shape
    return np.logical_and.reduce([p[y:y+h,x:x+w] for y in range(3) for x in range(3)])


def sphere_scores(clouds, mask, centers, radius):
    mask=erode(mask)&np.logical_and.reduce([np.isfinite(p).all(axis=2)&(p[:,:,2]>0)&(p[:,:,2]<2.) for p in clouds.values()])
    # Same observed interior pixels, same centers, same radius in both models.
    # Nearest center assigned from baseline; no post-hoc per-candidate mask.
    b=clouds['baseline'][mask]; dist=np.linalg.norm(b[:,None,:]-centers[None,:,:],axis=2); ids=dist.argmin(axis=1)
    if len(b)<100: raise ValueError('Insufficient sphere core points')
    return {m:stats(np.linalg.norm(p[mask]-centers[ids],axis=1)-radius) for m,p in clouds.items()}


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--input-dir',type=Path,default=Path(__file__).resolve().parent); p.add_argument('--output-dir',type=Path,default=Path.home()/'Downloads'); p.add_argument('--self-check',action='store_true'); a=p.parse_args()
    meta,depths,poses,spheres,sdata,rgb=load_inputs(a.input_dir)
    if a.self_check:
        # Offline numerical sanity check only, NOT a ROS runtime test.
        for s in SIDES:
            k=meta['sides'][s]['camera_info']['K']; d=depths[s]
            c={m:pinhole(d,[k[0],0,cx,0,k[4],cy,0,0,1]) for m,(cx,cy) in MODELS.items()}
            print(s,json.dumps(floor_scores(s,d,c,meta,poses)))
        print('Input and numerical scoring self-check passed (no ROS run).'); return 0
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from tf2_ros import Buffer, TransformListener
    tag=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    ns='/echo/s03/d5n_'+str(os.getpid()); out=a.output_dir/f'echo_s03_d5n_ros_gt_replay_{tag}.zip'
    a.output_dir.mkdir(parents=True,exist_ok=True); logsdir=a.output_dir/f'echo_s03_d5n_logs_{tag}'; logsdir.mkdir()
    rclpy.init(); node=Node('echo_s03_d5n_replay_'+str(os.getpid())); procs=[]; handles=[]; report={'study':'D5N-A actual ROS depth_image_proc archived GT replay','namespace':ns,'input_sha256':{f:hashlib.sha256((a.input_dir/f).read_bytes()).hexdigest() for f in (D4ZIP,POSES,D5ZIP)},'floor':{},'sphere':{},'limitations':['Archive replay, not live ROS transport/exposure timing. D4F floor masks are geometric candidates, not semantic labels. Both pose time domains scored. D5M one stationary front pose, synthetic depth. Registration uses current front static TF, checked against archived extrinsic. Not RTAB-Map end-to-end.']}; arrays={}; result={}; subs=[]
    def spawn(name,exe,remap):
        f=(logsdir/(name+'.log')).open('w'); handles.append(f)
        cmd=['ros2','run','depth_image_proc',exe,'--ros-args','-r','__ns:='+ns,'-r','__node:='+name,'-p','use_sim_time:=false']
        for src,dst in remap.items():cmd+=['-r',src+':='+dst]
        procs.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
    def save(key,msg): result[(key,msg.header.stamp.sec,msg.header.stamp.nanosec)]=msg
    def listen(key,typ,topic): subs.append(node.create_subscription(typ,topic,lambda msg,k=key:save(k,msg),qos_profile_sensor_data))
    def xyz(msg):
        if (msg.height,msg.width)!=(480,640):raise RuntimeError('Expected organized 640x480 cloud')
        offsets={f.name:f.offset for f in msg.fields if f.datatype==7 and f.count==1}
        if not all(k in offsets for k in 'xyz'):raise RuntimeError('Missing float32 xyz fields')
        endian='>' if msg.is_bigendian else '<'
        return np.stack([np.ndarray((msg.height,msg.width),dtype=endian+'f4',buffer=msg.data,offset=offsets[k],strides=(msg.row_step,msg.point_step)) for k in 'xyz'],axis=-1).copy()
    def read_depth(msg):
        if msg.encoding!='32FC1':raise RuntimeError('Unexpected depth encoding '+msg.encoding)
        return np.ndarray((msg.height,msg.width),dtype=('>' if msg.is_bigendian else '<')+'f4',buffer=msg.data,strides=(msg.step,4)).copy()
    try:
        tfbuf=Buffer(); tfl=TransformListener(tfbuf,node)
        end=time.monotonic()+8
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.15)
            if tfbuf.can_transform('echo_d455_front_color_optical_frame','echo_d455_front_depth_optical_frame',Time()):break
        tr=tfbuf.lookup_transform('echo_d455_front_color_optical_frame','echo_d455_front_depth_optical_frame',Time()).transform
        t=np.array([tr.translation.x,tr.translation.y,tr.translation.z]); q=np.array([tr.rotation.w,tr.rotation.x,tr.rotation.y,tr.rotation.z]); R=rotation(q)
        # Compare actual transform with known D5M local center correspondences (ROS optical coordinates).
        centers={s:np.array([r['sensors'][s]['center_camera_local_m'] for r in spheres['per_sphere']])*[1,-1,-1] for s in ('rgb','depth')}
        err=float(np.max(np.linalg.norm(centers['depth']@R.T+t-centers['rgb'],axis=1)))
        report['current_front_depth_to_rgb_tf']={'xyz_m':t.tolist(),'quat_wxyz':q.tolist(),'max_archived_center_transform_error_m':err}
        if err>1e-4: raise RuntimeError(f'Live front extrinsic differs from D5M by {err} m')
        print('Front extrinsic verified against D5M, max center discrepancy m:',err,flush=True)
        pub_depth=node.create_publisher(Image,ns+'/raw_depth',10)
        ci_pub={}
        for mode in MODELS:
            root=ns+'/'+mode
            for s in ('depth','rgb'):ci_pub[mode,s]=node.create_publisher(CameraInfo,root+'/'+s+'_info',10)
            spawn(mode+'_native','point_cloud_xyz_node',{'image_rect':ns+'/raw_depth','camera_info':root+'/depth_info','points':root+'/native_points'})
            spawn(mode+'_register','register_node',{'depth/image_rect':ns+'/raw_depth','depth/camera_info':root+'/depth_info','rgb/camera_info':root+'/rgb_info','depth_registered/image_rect':root+'/registered/image_rect','depth_registered/camera_info':root+'/registered/camera_info'})
            spawn(mode+'_registered','point_cloud_xyz_node',{'image_rect':root+'/registered/image_rect','camera_info':root+'/registered/camera_info','points':root+'/registered_points'})
            for k,typ,suffix in [('native',PointCloud2,'native_points'),('registered',PointCloud2,'registered_points'),('regdepth',Image,'registered/image_rect'),('reginfo',CameraInfo,'registered/camera_info')]:listen((mode,k),typ,root+'/'+suffix)
        def camera_info(source,frame,stamp,mode):
            ci=CameraInfo(); ci.header.frame_id=frame;ci.header.stamp=stamp;ci.height=480;ci.width=640;ci.distortion_model=source['distortion_model']; ci.d=list(source['D']);ci.k=list(source['K']);ci.r=list(source['R']);ci.p=list(source['P']);ci.binning_x=source.get('binning_x',0);ci.binning_y=source.get('binning_y',0)
            cx,cy=MODELS[mode];ci.k[2]=cx;ci.k[5]=cy;ci.p[2]=cx;ci.p[6]=cy
            return ci
        def replay(side,d,reg=False):
            source=meta['sides'][side]['camera_info']; stamp=node.get_clock().now().to_msg(); frame=source['frame_id']; rgbframe=frame.replace('_depth_','_color_'); msg=Image();msg.header.frame_id=frame;msg.header.stamp=stamp;msg.height=480;msg.width=640;msg.encoding='32FC1';msg.is_bigendian=0;msg.step=2560;msg.data=np.ascontiguousarray(d,dtype='<f4').tobytes(); result.clear()
            required=[(m,k) for m in MODELS for k in (('native','registered','regdepth','reginfo') if reg else ('native',))]
            def key(k):return (k,stamp.sec,stamp.nanosec)
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                if any(p.poll() is not None for p in procs):raise RuntimeError('A ROS child exited; inspect '+str(logsdir))
                for mode in MODELS:
                    ci_pub[mode,'depth'].publish(camera_info(source,frame,stamp,mode))
                    if reg:ci_pub[mode,'rgb'].publish(camera_info(source,rgbframe,stamp,mode))
                pub_depth.publish(msg)
                end=time.monotonic()+.4
                while time.monotonic()<end and rclpy.ok():rclpy.spin_once(node,timeout_sec=.04)
                if all(key(k) in result for k in required):break
            if not all(key(k) in result for k in required):raise RuntimeError('Missing same-stamp outputs: '+str([k for k in required if key(k) not in result])+' logs='+str(logsdir))
            data={k:result[key(k)] for k in required}
            for mode in MODELS:
                if data[mode,'native'].header.frame_id!=frame:raise RuntimeError('Wrong native cloud frame')
                if reg:
                    if any(data[mode,k].header.frame_id!=rgbframe for k in ('registered','regdepth','reginfo')):raise RuntimeError('Wrong registered frame')
                    ci=data[mode,'reginfo'];cx,cy=MODELS[mode]
                    if not np.allclose([ci.k[2],ci.k[5],ci.p[2],ci.p[6]],[cx,cy,cx,cy],atol=1e-7,rtol=0):raise RuntimeError('Registered CameraInfo did not preserve candidate model')
            return data
        def consumer_check(d,cloud,k):
            expected=pinhole(d,k);valid=np.isfinite(d)&(d>0);actual=np.isfinite(cloud).all(axis=2)&(cloud[:,:,2]>0)
            common=valid&actual
            if not common.any():raise RuntimeError('No valid cloud points')
            diff=np.linalg.norm(cloud[common]-expected[common],axis=1)
            return {'valid_mask_disagreements':int(np.count_nonzero(valid!=actual)),'max_xyz_error_to_expected_m':float(diff.max()),'rms_xyz_error_to_expected_m':float(np.sqrt(np.mean(diff*diff)))}
        for side in SIDES:
            d=depths[side];data=replay(side,d); clouds={m:xyz(data[m,'native']) for m in MODELS}; checks={}
            for m,(cx,cy) in MODELS.items():
                k=list(meta['sides'][side]['camera_info']['K']);k[2]=cx;k[5]=cy;checks[m]=consumer_check(d,clouds[m],k);arrays[side+'_'+m+'_native_xyz']=clouds[m]
            rec=floor_scores(side,d,clouds,meta,poses);rec['consumer_check']=checks;report['floor'][side]=rec
            print(side,'floor RMSE mm (sim manager):', {m:round(v['rmse_mm'],6) for m,v in rec['simulation_manager_time_s']['models'].items()},flush=True)
        d=sdata['native_depth_visible'].copy()
        # Replicator background is invalid (inf); encode invalid depth as NaN for ROS replay.
        # All positive finite sphere depths unchanged; record this explicit normalization.
        d[~np.isfinite(d)|(d<=0)]=np.nan
        data=replay('front',d,True); native={m:xyz(data[m,'native']) for m in MODELS}; reg={m:xyz(data[m,'registered']) for m in MODELS}; rd={m:read_depth(data[m,'regdepth']) for m in MODELS}
        for m,(cx,cy) in MODELS.items():
            k=list(meta['sides']['front']['camera_info']['K']);k[2]=cx;k[5]=cy;ci=data[m,'reginfo'];report['sphere'].setdefault('consumer_checks',{})[m]={'native':consumer_check(d,native[m],k),'registered':consumer_check(rd[m],reg[m],list(ci.k)),'registered_K':list(ci.k),'registered_P':list(ci.p)}
            arrays[m+'_sphere_native_xyz']=native[m];arrays[m+'_sphere_registered_xyz']=reg[m];arrays[m+'_sphere_registered_depth']=rd[m]
        report['sphere']['native_radial_surface']=sphere_scores(native,sdata['observed_depth_spheres'],centers['depth'],spheres['radius_m'])
        report['sphere']['registered_radial_surface']=sphere_scores(reg,sdata['observed_rgb_magenta'],centers['rgb'],spheres['radius_m'])
        b,c=rd.values(); vb=np.isfinite(b)&(b>0);vc=np.isfinite(c)&(c>0);common=vb&vc
        report['sphere']['registration']={'valid_counts':{m:int((np.isfinite(v)&(v>0)).sum()) for m,v in rd.items()},'mask_disagreement':int((vb!=vc).sum()),'common_valid':int(common.sum()),'exact_fraction':float(np.mean(b[common]==c[common])) if common.any() else None}
        print('Sphere native radial surface:',report['sphere']['native_radial_surface'],flush=True);print('Sphere RGB-frame registered radial surface:',report['sphere']['registered_radial_surface'],flush=True)
        report['status']='complete; inspect metrics before declaring calibration pass'
        print('Completed actual ROS replay; no production topics published.',flush=True)
    except Exception as e:
        report['status']='incomplete';report['error']=repr(e);print('INCOMPLETE:',repr(e),flush=True)
    finally:
        for proc in procs:
            if proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
        for proc in procs:
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait()
        for f in handles:f.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as z:
            z.writestr('report.json',json.dumps(report,indent=2,allow_nan=False));buf=io.BytesIO();np.savez_compressed(buf,**arrays);z.writestr('actual_ros_clouds.npz',buf.getvalue());z.writestr('d5m_rgb_reference.png',rgb);z.write(Path(__file__),'replay_source.py')
            for f in logsdir.glob('*.log'):z.write(f,'logs/'+f.name)
        print('RESULT ZIP:',out,flush=True)
    return 0 if report['status'].startswith('complete') else 2

if __name__=='__main__':raise SystemExit(main())
