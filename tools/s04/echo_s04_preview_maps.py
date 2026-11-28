#!/usr/bin/env python3
"""Render existing S04 voxel PLYs; no ROS, no raw bag reads, no modifications to evidence."""
import argparse
import json
from pathlib import Path
import tempfile
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

NAMES = ('one_d455', 'four_d455', 'lidar')
LABELS = ('1 D455', '4 D455s', '3D LiDAR')
DT = np.dtype([(k, '<f4') for k in 'xyz'] + [(k, 'u1') for k in ('red', 'green', 'blue')])
PROPS = ['property float x','property float y','property float z','property uchar red','property uchar green','property uchar blue']

def read_ply(path, count):
    with path.open('rb') as f:
        header = []
        for _ in range(64):
            line = f.readline(1024).decode('ascii').strip()
            if not line: raise ValueError(f'Incomplete header: {path}')
            header.append(line)
            if line == 'end_header': break
        if (header[:2] != ['ply','format binary_little_endian 1.0'] or header[-1] != 'end_header' or
            [s for s in header if s.startswith('element ')] != [f'element vertex {count}'] or
            [s for s in header if s.startswith('property ')] != PROPS):
            raise ValueError(f'Unexpected PLY schema/count: {path}')
        if path.stat().st_size - f.tell() != count * DT.itemsize:
            raise ValueError(f'Unexpected PLY payload size: {path}')
        a = np.fromfile(f, dtype=DT, count=count)
    p = np.column_stack([a[k] for k in 'xyz'])
    if len(p) != count or not np.isfinite(p).all(): raise ValueError(f'Invalid points: {path}')
    return p

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir', type=Path)
    d = parser.parse_args().map_dir.resolve(strict=True)
    audit = json.loads((d/'audit.json').read_text())
    if audit['status'] != 'PILOT RECONSTRUCTION COMPLETE': raise ValueError('Reconstruction not marked complete')
    arrays = [read_ply(d/(k+'.ply'), audit['maps'][k]['occupied_voxels']) for k in NAMES]
    allp = np.concatenate(arrays)
    lo, hi = allp.min(0), allp.max(0)
    az, el = np.deg2rad([40,28]); right = np.array([np.cos(az),-np.sin(az),0]); up = np.array([-np.sin(az)*np.sin(el),-np.cos(az)*np.sin(el),np.cos(el)]); normal = np.cross(right,up)
    u,v = allp@right,allp@up
    fig,axes = plt.subplots(2,3,figsize=(15,10),layout='constrained')
    for j,(p,label) in enumerate(zip(arrays,LABELS)):
        ax=axes[0,j]; ix=np.argsort(p[:,2]); sc=ax.scatter(p[ix,0],p[ix,1],c=p[ix,2],vmin=lo[2],vmax=hi[2],s=.7,cmap='viridis',linewidths=0); ax.set(xlim=(lo[0]-.2,hi[0]+.2),ylim=(lo[1]-.2,hi[1]+.2),aspect='equal',title=f'{label} | {len(p):,} voxels',xlabel='odom x (m)',ylabel='odom y (m)')
        ax=axes[1,j]; ix=np.argsort(p@normal); ax.scatter(p[ix]@right,p[ix]@up,c=p[ix,2],vmin=lo[2],vmax=hi[2],s=.7,cmap='viridis',linewidths=0); ax.set(xlim=(u.min()-.2,u.max()+.2),ylim=(v.min()-.2,v.max()+.2),aspect='equal',title='Oblique | height colors',xlabel='Projected horizontal (m)',ylabel='Projected vertical (m)')
    fig.colorbar(sc,ax=axes.ravel().tolist(),label='Height in odom (m)',shrink=.75)
    fig.suptitle('ECHO S04 | Matched odometry-projected voxel centers (not optimized SLAM)')
    out=Path(tempfile.mkdtemp(prefix='s04_preview_',dir=d)); preview=out/'preview.png'; fig.savefig(preview,dpi=150); plt.close(fig)
    archive=out.with_suffix('.zip')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for file in [d/'audit.json',*[d/(k+'.ply') for k in NAMES],preview]: z.write(file,file.name)
    print('PASS: all three PLY counts and payload sizes verified')
    print('PREVIEW:',preview)
    print('UPLOAD:',archive)

if __name__=='__main__':main()
