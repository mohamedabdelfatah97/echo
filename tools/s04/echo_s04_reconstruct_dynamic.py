#!/usr/bin/env python3
"""Read-only S04 dynamic audit and matched odometry-projected maps (not SLAM).
Run with ROS Jazzy and the recorded rtabmap_msgs installed. No ROS nodes/publishers.
"""
import argparse, collections, json, math, sys, tempfile, time, traceback, zipfile
from pathlib import Path
import numpy as np

RGBD='/echo/s03/rgbd_images'; LIDAR='/front_3d_lidar/lidar_points'; ODOM='/chassis/odom'
SIDES=('front','rear','left','right'); RANGE=(.5,10.); VOXEL=.05; STRIDE=4

def require(ok,msg):
    if not ok: raise RuntimeError(msg)

def stamp(s): return s.sec+s.nanosec*1e-9

def matrix(p,q):
    q=np.asarray(q,dtype=float); require(np.isfinite(q).all() and abs(np.linalg.norm(q)-1)<.01,'Invalid quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    t=np.eye(4); t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]; t[:3,3]=p
    require(np.isfinite(t).all(),'Invalid transform'); return t

def xyz(p): return [p.x,p.y,p.z]
def quat(q): return [q.x,q.y,q.z,q.w]
def tfmat(t): return matrix(xyz(t.translation),quat(t.rotation))
def apply(p,t): return p@t[:3,:3].T+t[:3,3]

def interp(ts,ps,qs,t,max_gap=.15):
    i=np.searchsorted(ts,t)
    if i<len(ts) and abs(ts[i]-t)<1e-8: return matrix(ps[i],qs[i])
    if i==0 or i==len(ts) or ts[i]-ts[i-1]>max_gap: return None
    u=(t-ts[i-1])/(ts[i]-ts[i-1]); a=qs[i-1]; b=qs[i]
    dot=float(a@b)
    if dot<0: b=-b; dot=-dot
    dot=np.clip(dot,-1,1)
    if dot>.9995: q=a+u*(b-a)
    else:
        th=math.acos(dot); q=(math.sin((1-u)*th)*a+math.sin(u*th)*b)/math.sin(th)
    return matrix((1-u)*ps[i-1]+u*ps[i],q/np.linalg.norm(q))

def lookup(edges,target,source):
    todo=[(source,np.eye(4))]; seen={source}
    for node,t in todo:
        if node==target:return t
        for (parent,child),edge in edges.items():
            if child==node: nxt,step=parent,edge
            elif parent==node: nxt,step=child,np.linalg.inv(edge)
            else: continue
            if nxt not in seen:seen.add(nxt);todo.append((nxt,step@t))
    raise RuntimeError(f'No recorded fixed TF {target} <- {source}; edges: {list(edges)}')

def frame(s): return s.strip('/')

def image_array(im,dtype,channels=1):
    dt=np.dtype(dtype); require(im.height>0 and im.width>0,'Empty image')
    require(im.step>=im.width*channels*dt.itemsize and len(im.data)==im.step*im.height,'Invalid image byte layout')
    dt=dt.newbyteorder('>' if im.is_bigendian else '<')
    return np.ndarray((im.height,im.width,channels),dtype=dt,buffer=bytes(im.data),strides=(im.step,channels*dt.itemsize,dt.itemsize))

def camera_meta(a):
    require(len(a.rgbd_images)==4,'RGBD aggregate does not contain four cameras')
    res=[]
    for side,c in zip(SIDES,a.rgbd_images):
        rgb,dep,info=c.rgb,c.depth,c.rgb_camera_info
        f=frame(info.header.frame_id)
        require(side in f and 'optical' in f and ('color' in f or 'rgb' in f),f'Unexpected {side} RGB optical frame: {f}')
        require(frame(rgb.header.frame_id)==f and frame(dep.header.frame_id)==f,f'Registration frame mismatch {side}: {rgb.header.frame_id}, {dep.header.frame_id}, {f}')
        require((rgb.width,rgb.height)==(640,480)==(dep.width,dep.height)==(info.width,info.height),f'Unexpected image dimensions {side}')
        require(rgb.encoding in ('rgb8','bgr8') and dep.encoding in ('32FC1','16UC1'),f'Unexpected image encodings {side}: {rgb.encoding}, {dep.encoding}')
        k=np.asarray(info.k).reshape(3,3); p=np.asarray(info.p).reshape(3,4)
        require(np.allclose(k,p[:,:3],atol=1e-5) and np.allclose(p[:,3],0,atol=1e-5),'Unexpected projection model')
        require(np.allclose([k[0,0],k[1,1],k[0,2],k[1,2]],[317.04311998,317.04311998,319.5,239.5],atol=.01),'Unexpected coupled_xy intrinsics')
        require(abs(stamp(rgb.header.stamp)-stamp(dep.header.stamp))<1e-6,f'Unsynchronized RGB/depth {side}')
        res.append((stamp(dep.header.stamp),f,k,rgb.encoding,dep.encoding))
    return res

def cloud_xyz(m):
    fields={f.name:f for f in m.fields}; require(all(k in fields for k in 'xyz'),'LiDAR XYZ fields missing')
    require(m.width>0 and m.height>0 and m.row_step>=m.width*m.point_step and len(m.data)==m.row_step*m.height,'Invalid PointCloud2 layout')
    endian='>' if m.is_bigendian else '<'; arr=[]; raw=bytes(m.data)
    for k in 'xyz':
        f=fields[k]; require(f.datatype in (7,8) and f.count==1,'Unsupported LiDAR XYZ datatype')
        dt=np.dtype(endian+('f4' if f.datatype==7 else 'f8')); require(f.offset+dt.itemsize<=m.point_step,'Invalid field offset')
        arr.append(np.ndarray((m.height,m.width),dtype=dt,buffer=raw,offset=f.offset,strides=(m.row_step,m.point_step)).reshape(-1))
    return np.column_stack(arr)

def range_filter(p):
    r=np.sum(p*p,axis=1); return np.isfinite(p).all(axis=1)&(r>=RANGE[0]**2)&(r<=RANGE[1]**2)

def camera_points(c,meta):
    _,_,k,rgbenc,depenc=meta
    z=image_array(c.depth,'f4' if depenc=='32FC1' else 'u2')[::STRIDE,::STRIDE,0].astype(np.float32)
    if depenc=='16UC1':z*=.001
    v,u=np.mgrid[0:c.depth.height:STRIDE,0:c.depth.width:STRIDE]; p=np.stack([(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z],axis=-1).reshape(-1,3)
    good=range_filter(p)&(p[:,2]>0); colors=image_array(c.rgb,'u1',3)[::STRIDE,::STRIDE].reshape(-1,3)
    if rgbenc=='bgr8':colors=colors[:,::-1]
    return p[good],colors[good]

class Voxels:
    def __init__(self):self.data={};self.points=0
    def add(self,p,rgb=None):
        self.points+=len(p)
        if not len(p):return
        keys=np.floor(p/VOXEL).astype(np.int32); keys,ix=np.unique(keys,axis=0,return_index=True)
        for j,k in enumerate(keys):
            key=tuple(map(int,k))
            if key not in self.data:self.data[key]=tuple(map(int,rgb[ix[j]])) if rgb is not None else (175,175,175)
    def arrays(self):
        require(bool(self.data),'Empty reconstructed map')
        keys=np.array(list(self.data),dtype=np.int32); return (keys.astype(np.float32)+.5)*VOXEL,np.array(list(self.data.values()),dtype=np.uint8)
    def save(self,path):
        p,c=self.arrays(); a=np.empty(len(p),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')]);
        for i,k in enumerate('xyz'):a[k]=p[:,i]
        for i,k in enumerate(('red','green','blue')):a[k]=c[:,i]
        with path.open('xb') as f:
            f.write(('ply\nformat binary_little_endian 1.0\ncomment ECHO common odometry projected voxel centers, not optimized SLAM\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'%len(p)).encode());a.tofile(f)
        return {'input_points':self.points,'occupied_voxels':len(p),'bounds_min_m':p.min(0).tolist(),'bounds_max_m':p.max(0).tolist()}

def stats(times,start,end):
    t=np.array(times); r=np.sort(t[(t>=start)&(t<=end)]); g=np.diff(r)
    return dict(total=len(t),in_route=len(r),first_route=float(r[0]) if len(r) else None,last_route=float(r[-1]) if len(r) else None,duplicate_stamps=int(np.sum(g==0)),backward_in_record_order=int(np.sum(np.diff(t)<-1e-8)),median_gap_s=float(np.median(g)) if len(g) else None,max_gap_s=float(max(g)) if len(g) else None)

class ActorAudit:
    """Time-dependent spatial proxy, not actor segmentation or visibility truth."""
    def __init__(self, rows, align):
        self.ts=np.array([r['sim_time_s'] for r in rows]); self.world_from_odom=np.linalg.inv(align)
        require(len(rows)>2 and np.all(np.diff(self.ts)>0),'Invalid actor GT timeline')
        ids={a['id'] for a in rows[0]['actors']}; require(len(ids)==4,'Expected four actors')
        self.ids=sorted(ids); self.tracks={}; self.kinds={}; self.results={}; self.unmatched=collections.Counter()
        for r in rows: require(len(r['actors'])==4 and {a['id'] for a in r['actors']}==ids,'Actor identity changed')
        for a in rows[0]['actors']:
            k=a['id']; self.kinds[k]=a['kind']; self.tracks[k]=np.array([next(x['xyz_m'] for x in r['actors'] if x['id']==k) for r in rows]); require(np.isfinite(self.tracks[k]).all(),'Invalid actor position')
            self.results[k]={name:dict(points=0,frames=set(),voxels=set()) for name in (*SIDES,'one_d455','four_d455','lidar')}
    def sample(self,p,t,source,group):
        i=np.searchsorted(self.ts,t)
        if i<len(self.ts) and abs(self.ts[i]-t)<1e-8: j=i; u=0.
        elif i>0 and i<len(self.ts) and self.ts[i]-self.ts[i-1]<=.15: j=i-1; u=(t-self.ts[j])/(self.ts[i]-self.ts[j])
        else: self.unmatched[source]+=1; return
        world=apply(p,self.world_from_odom)
        for k in self.ids:
            track=self.tracks[k]; center=track[j]+u*(track[i]-track[j]); d=world-center
            human=self.kinds[k]=='IRA_Character'; radius=.6 if human else 1.0; top=2.1 if human else 1.6
            mask=(np.sum(d[:,:2]**2,axis=1)<=radius**2)&(d[:,2]>=.10)&(d[:,2]<=top)
            if not np.any(mask): continue
            keys={tuple(map(int,v)) for v in np.unique(np.floor(p[mask]/VOXEL).astype(np.int32),axis=0)}
            names=[source] if source=='lidar' else [source,'four_d455']+(['one_d455'] if source=='front' else [])
            for name in names:
                r=self.results[k][name]; r['points']+=int(mask.sum()); r['frames'].add(group); r['voxels'].update(keys)
    def summary(self):
        return {'policy':'Time-interpolated actor-root cylinders in simulator world, aligned once to odometry. Humans radius 0.6m height 0.1–2.1m; robots radius 1m height 0.1–1.6m. Heuristic candidate returns only, may include nearby static surfaces, no segmentation or proven line of sight. Frames for four cameras count aggregate IDs, not camera exposures. Keys are integer odom voxel indices.', 'unmatched_actor_gt_frames':dict(self.unmatched),'actors':{k:{n:{'point_samples':r['points'],'observation_groups':len(r['frames']),'candidate_voxel_count':len(r['voxels']),'candidate_voxel_keys':sorted(r['voxels'])} for n,r in v.items()} for k,v in self.results.items()}}

def main(trial,out,report):
    import rosbag2_py as rb
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    events=[json.loads(l) for l in (trial/'route_events.jsonl').read_text().splitlines()]; start=next(x['sim_s'] for x in events if x['kind']=='route_start'); end=next(x['sim_s'] for x in events if x['kind']=='route_end')
    require(end>start,'Bad route interval');report['route_sim_s']=[start,end]
    def read(topics):
        reader=rb.SequentialReader();reader.open(rb.StorageOptions(uri=str(trial/'bag'),storage_id='mcap'),rb.ConverterOptions('',''))
        types={x.name:x.type for x in reader.get_all_topics_and_types()};require(set(topics)<=types.keys(),f'Missing topics: {set(topics)-types.keys()}')
        cls={t:get_message(types[t]) for t in topics}; reader.set_filter(rb.StorageFilter(topics=list(topics)))
        while reader.has_next():
            t,b,wall=reader.read_next();yield t,deserialize_message(b,cls[t]),wall
    print('1/3 Auditing recorded poses, TF, image headers and cloud headers...',flush=True)
    times=collections.defaultdict(list);counts=collections.Counter();edges={}; dynamic={}; poses=[]; cameras=[];lidars=[];skew=[];schemas={};bad=collections.Counter();bad_examples={};clock=[]
    def record_bad(k,e):
        bad[k]+=1;bad_examples.setdefault(k,str(e))
    for t,m,w in read([RGBD,LIDAR,ODOM,'/tf','/tf_static','/clock','/scan']):
        i=counts[t];counts[t]+=1
        if t==ODOM:
            require(frame(m.header.frame_id)=='odom' and frame(m.child_frame_id)=='base_link','Unexpected odometry frame');s=stamp(m.header.stamp);p=m.pose.pose;poses.append((s,xyz(p.position),quat(p.orientation)));times[t].append(s)
        elif t in ('/tf','/tf_static'):
            for tf in m.transforms:
                k=(frame(tf.header.frame_id),frame(tf.child_frame_id));v=tfmat(tf.transform)
                if t=='/tf_static':
                    if k in edges:require(np.allclose(edges[k],v,atol=1e-5),f'Changing static TF {k}')
                    edges[k]=v
                elif k!=('odom','base_link'):
                    if k not in dynamic:dynamic[k]=[v,True,0]
                    dynamic[k][1]&=bool(np.allclose(dynamic[k][0],v,atol=1e-5));dynamic[k][2]+=1
        elif t=='/clock': clock.append(stamp(m.clock));times[t].append(clock[-1])
        elif t=='/scan':times[t].append(stamp(m.header.stamp))
        elif t==RGBD:
            s=stamp(m.header.stamp);times[t].append(s)
            try:
                meta=camera_meta(m); ct=[x[0] for x in meta];spread=max(ct)-min(ct);skew.append(spread)
                require(spread<=.050001,f'Aggregate skew {spread:.6f}s exceeds 50ms')
                require(abs(s-max(ct))<=.051,'Unexpected aggregate timestamp')
                sig=[(x[1],x[3],x[4]) for x in meta]
                require('cameras' not in schemas or schemas['cameras']==sig,'Camera schema changed');schemas['cameras']=sig;cameras.append((i,s,ct))
            except RuntimeError as e:record_bad('rgbd',e)
        elif t==LIDAR:
            s=stamp(m.header.stamp);times[t].append(s);lidars.append((i,s));f=frame(m.header.frame_id)
            require('lidar_frame' not in schemas or schemas['lidar_frame']==f,'LiDAR frame changed');schemas['lidar_frame']=f
    report.update(counts=dict(counts),streams={t:stats(v,start,end) for t,v in times.items()},schemas=schemas,invalid=dict(bad),invalid_examples=bad_examples,aggregate_skew_max_s=max(skew) if skew else None)
    require(len(clock)>2 and np.all(np.diff(clock)>=-1e-8),'Recorded clock reset')
    require(len(cameras)>0 and len(lidars)>0,'No valid camera or lidar frames')
    # Use static transforms, plus explicitly verified constant repeated /tf edges (not odom).
    for k,(v,constant,n) in dynamic.items():
        if constant and n>=2:
            if k in edges:require(np.allclose(edges[k],v,atol=1e-5),f'Conflicting fixed TF {k}')
            else:edges[k]=v
    report['fixed_tf_edges']={f'{a} <- {b}':v.tolist() for (a,b),v in edges.items()}
    ext={f:lookup(edges,'base_link',f) for f,_,_ in schemas['cameras']};lt=lookup(edges,'base_link',schemas['lidar_frame'])
    report['base_from_sensor']={**{f:v.tolist() for f,v in ext.items()},schemas['lidar_frame']:lt.tolist()}
    # Keep one pose per stamp; reject conflicting duplicate poses.
    require(all(b[0]>=a[0]-1e-8 for a,b in zip(poses,poses[1:])),'Odometry timestamp reset')
    pd={}
    for s,p,q in poses:
        if s in pd:require(np.allclose(matrix(p,q),matrix(*pd[s]),atol=1e-5),'Conflicting duplicate odometry')
        pd[s]=(p,q)
    ts=np.array(sorted(pd));ps=np.array([pd[t][0] for t in ts]);qs=np.array([pd[t][1] for t in ts]);qs/=np.linalg.norm(qs,axis=1)[:,None]
    def at(t):return interp(ts,ps,qs,t)
    # Align simulator GT once to odometry; no trajectory fitting.
    gt=[json.loads(l) for l in (trial/'physics.jsonl').read_text().splitlines()];require(not any(x.get('type')=='error' for x in gt),'GT logger error'); all_gt=[x for x in gt if x.get('type')=='pose']; gt=[x for x in all_gt if start<=x['sim_time_s']<=end]
    pairs=[]
    for x in gt:
        pose=at(x['sim_time_s']);q=x['quat_wxyz'];g=matrix(x['xyz_m'],q[1:]+q[:1]);
        if pose is not None:pairs.append((pose,g))
    require(len(pairs)>10,'Insufficient GT/odom matches');align=pairs[0][0]@np.linalg.inv(pairs[0][1]);err=[]
    for p,g in pairs:err.append(float(np.linalg.norm((p-align@g)[:2,3])))
    report['gt_odom_xy_initial_alignment']={'matched':len(err),'rmse_m':float(np.sqrt(np.mean(np.square(err)))),'max_m':max(err),'note':'Simulator-derived odom consistency; no clock-offset optimization'}
    actor_audit=ActorAudit(all_gt,align)
    # Match at most one lidar scan per 0.2 s bin to nearest unused valid RGBD aggregate.
    cams=sorted((x for x in cameras if start<=min(x[2]) and max(x[2])<=end and all(at(t) is not None for t in x[2])),key=lambda x:x[1]);ls=sorted((x for x in lidars if start<=x[1]<=end and at(x[1]) is not None),key=lambda x:x[1]); require(cams and ls,'No time-aligned frames')
    ct=np.array([x[1] for x in cams]);used=set();bins=set();selected=[]
    for li,s in ls:
        b=int((s-start)/.2)
        if b in bins:continue
        idx=np.searchsorted(ct,s);near=sorted(range(max(0,idx-4),min(len(ct),idx+4)),key=lambda j:abs(ct[j]-s))
        for j in near:
            ci,cs,sub=cams[j]
            if ci not in used and abs(cs-s)<=.10:
                selected.append((ci,li,cs,s));used.add(ci);bins.add(b);break
    report['matching']={'pairs':len(selected),'max_stamp_difference_s':max((abs(a-b) for _,_,a,b in selected),default=None),'max_pair_gap_s':max(np.diff([x[3] for x in selected]),default=None),'route_fraction_bins':len(bins)/math.ceil((end-start)/.2),'policy':'One pair per 0.2s bin, nearest unused RGBD aggregate <=0.10s, each sensor projected at own stamp'}
    require(len(selected)>=30 and report['matching']['route_fraction_bins']>=.5,'Insufficient matched observations for pilot')
    (out/'audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report['matching'],indent=2),flush=True)
    print('2/3 Projecting matched observations into common odometry frame...',flush=True)
    ci={x[0] for x in selected};li={x[1] for x in selected};counts2=collections.Counter();maps={k:Voxels() for k in ('one_d455','four_d455','lidar')};processed=collections.Counter()
    for t,m,w in read([RGBD,LIDAR]):
        i=counts2[t];counts2[t]+=1
        if t==RGBD and i in ci:
            for j,(c,meta) in enumerate(zip(m.rgbd_images,camera_meta(m))):
                p,rgb=camera_points(c,meta);require(len(p)>0,'Empty camera observation after range filter');p=apply(p,at(meta[0])@ext[meta[1]]);maps['four_d455'].add(p,rgb);actor_audit.sample(p,meta[0],SIDES[j],i)
                if j==0:maps['one_d455'].add(p,rgb)
            processed[t]+=1
        elif t==LIDAR and i in li:
            p=cloud_xyz(m);p=p[range_filter(p)];require(len(p)>0,'Empty LiDAR observation after range filter');p=apply(p,at(stamp(m.header.stamp))@lt);maps['lidar'].add(p);actor_audit.sample(p,stamp(m.header.stamp),'lidar',i);processed[t]+=1
    require(processed[RGBD]==processed[LIDAR]==len(selected),'Incomplete second pass')
    require(maps['one_d455'].data.keys()<=maps['four_d455'].data.keys(),'Front-camera subset invariant failed')
    report['maps']={k:m.save(out/(k+'.ply')) for k,m in maps.items()};report['parameters']={'euclidean_sensor_range_m':RANGE,'camera_pixel_stride':STRIDE,'lidar_point_stride':1,'voxel_m':VOXEL,'frame':'odom','note':'Geometric occupied voxels, not free-space or accuracy; no SLAM graph optimization'}
    report['actor_proximity_audit']=actor_audit.summary()
    report['world_to_odom_initial_alignment']=align.tolist()
    print('3/3 Saving maps and actor-proximity audit...',flush=True)
    report['status']='PILOT RECONSTRUCTION COMPLETE';print(json.dumps(report['maps'],indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);a=p.parse_args();trial=a.trial.resolve(strict=True);out=Path(tempfile.mkdtemp(prefix='s04_geometric_dynamic_',dir=trial));report={'status':'INCOMPLETE','trial':str(trial)};rc=0
    try:main(trial,out,report)
    except Exception as e:report['status']='FAILED';report['error']=str(e);(out/'error.txt').write_text(traceback.format_exc());traceback.print_exc();rc=1
    finally:
        (out/'audit.json').write_text(json.dumps(report,indent=2));z=out.with_name(out.name+'_summary.zip')
        with zipfile.ZipFile(z,'x',zipfile.ZIP_DEFLATED) as f:
            for x in out.iterdir():
                if x.suffix in ('.json','.png','.txt','.ply'):f.write(x,x.name)
        print('LOCAL MAPS:',out);print('UPLOAD:',z);sys.exit(rc)
