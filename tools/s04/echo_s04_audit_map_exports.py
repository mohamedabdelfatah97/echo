#!/usr/bin/env python3
"""Read-only summary-archive audit; NumPy + SciPy; no ROS or simulator."""
import argparse, io, json, zipfile
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

SENSORS=('one_d455','four_d455','lidar')
def ply(z, name):
    raw=z.read(name); head, sep, body=raw.partition(b'end_header\n')
    assert sep and b'format binary_little_endian 1.0' in head
    lines=head.decode().splitlines(); n=int(next(s.split()[2] for s in lines if s.startswith('element vertex ')))
    props=[]; active=False; types={'float':'<f4','double':'<f8','uchar':'u1','uint':'<u4','int':'<i4','char':'i1','ushort':'<u2','short':'<i2'}
    for line in lines:
        f=line.split()
        if f and f[0]=='element': active=f[1]=='vertex'
        if active and f[0]=='property': props.append((f[2],types[f[1]]))
    a=np.frombuffer(body,dtype=np.dtype(props),count=n)
    p=np.column_stack([a[k] for k in ('x','y','z')]).astype(float)
    assert np.isfinite(p).all()
    return p

def keys(p): return {tuple(k) for k in np.rint(p/.05-.5).astype(int)}
def rms(x): return float(np.sqrt(np.mean(np.asarray(x)**2)))
def load(path):
    with zipfile.ZipFile(path) as z:
        a=json.loads(z.read('audit.json')); maps={s:ply(z,s+'.ply') for s in SENSORS}
        for s,p in maps.items(): assert len(p)==a['maps'][s]['occupied_voxels']
    return a,maps

def geometric(a,m):
    bands={s:np.histogram(p[:,2],bins=[-np.inf,.1,1.5,2.8,np.inf])[0].tolist() for s,p in m.items()}
    common={s:p[(p[:,2]>=.1)&(p[:,2]<2.8)] for s,p in m.items()}
    nn={}
    for s in SENSORS[:2]:
        for x,y in [(s,'lidar'),('lidar',s)]:
            d=cKDTree(common[y]).query(common[x])[0]; nn[x+' -> '+y]={'median_m':float(np.median(d)), 'within_10cm_pct':float(100*np.mean(d<=.100001))}
    return dict(matching=a['matching'],maps=a['maps'],height_bands=bands,nn_common_height=nn,one_subset_four=keys(m['one_d455'])<=keys(m['four_d455']),gt_odom=a['gt_odom_xy_initial_alignment'])

def dynamic(a,m,static):
    actors=a['actor_proximity_audit']['actors']; out={}
    for s in SENSORS:
        records=[v[s] for v in actors.values()]; ks={tuple(k) for v in records for k in v['candidate_voxel_keys']}; p=(np.asarray(sorted(ks),dtype=float)+.5)*.05
        d=cKDTree(static[s]).query(p)[0] if len(p) else np.array([])
        out[s]=dict(point_samples=sum(v['point_samples'] for v in records),candidate_voxels=len(ks),fraction_pct=100*len(ks)/len(m[s]),beyond_static_10cm=int(np.sum(d>.100001)))
    out['per_camera']={s:sum(v[s]['point_samples'] for v in actors.values()) for s in ('front','rear','left','right')}
    return out

def yaw(q):
    x,y,z,w=q.T; return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
def wrap(x): return (x+np.pi)%(2*np.pi)-np.pi

def native(path):
    out={}
    with zipfile.ZipFile(path) as z:
        rows=[json.loads(l) for l in z.read('physics.jsonl').splitlines()]; rows=[r for r in rows if r.get('type')=='pose']; t=np.array([r['sim_time_s'] for r in rows]); p=np.array([r['xyz_m'] for r in rows]); q=np.array([r['quat_wxyz'] for r in rows])[:,[1,2,3,0]]; y=np.unwrap(yaw(q))
        for s in SENSORS:
            out[s]={}
            for mode in ('odometry','optimized'):
                x=np.loadtxt(io.BytesIO(z.read(f'{s}/{mode}_poses.txt'))); ts=x[:,0]; assert ts.min()>=t.min() and ts.max()<=t.max()
                g=np.column_stack([np.interp(ts,t,p[:,i]) for i in range(3)]); gy=np.interp(ts,t,y); py=yaw(x[:,4:8]); angle=py[0]-gy[0]; c,b=np.cos(angle),np.sin(angle); xy=(g[:,:2]-g[0,:2])@np.array([[c,b],[-b,c]])+x[0,1:3]; e=np.linalg.norm(x[:,1:3]-xy,axis=1)
                out[s][mode]={'count':len(ply(z,f'{s}/{mode}_cloud.ply')),'nodes':len(x),'xy_rmse_mm':1000*rms(e),'xy_max_mm':1000*float(e.max()),'yaw_rmse_deg':float(np.degrees(rms(wrap(py-gy-angle))))}
    return out

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('archives',type=Path,help='directory containing the four named summary ZIPs'); args=p.parse_args(); d=args.archives
    a,s=load(d/'ECHO_S04_three_maps.zip'); b,e=load(d/'ECHO_S04_extended_three_maps.zip'); c,m=load(d/'ECHO_S04_dynamic_three_maps.zip')
    out={'static_short':geometric(a,s),'static_extended':geometric(b,e),'dynamic':geometric(c,m),'dynamic_candidates':dynamic(c,m,s),'native_exports':native(d/'echo_s04_rtabmap_export_Im2WNU_summary.zip')}
    print(json.dumps(out,indent=2))
if __name__=='__main__':main()
