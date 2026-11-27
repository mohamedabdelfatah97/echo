#!/usr/bin/env python3
"""Read D7 JSONL logs; compare simulator and ROS pose without fitting the full path."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('trial', type=Path)
args = parser.parse_args()

def read(name):
    with (args.trial / name).open() as f:
        return [json.loads(line) for line in f]

physics, ros = read('physics.jsonl'), read('ros.jsonl')
a = [x for x in physics if x['type'] == 'pose']
b = [x for x in ros if x['type'] == 'odom']
assert physics[-1]['type'] == ros[-1]['type'] == 'summary'
assert all(x['frame_id'] == 'odom' and x['child_frame_id'] == 'base_link' for x in b)
t = np.array([x['simulation_manager_time_s'] for x in a])
s = np.array([x['stamp_ns']/1e9 for x in b])
assert np.all(np.diff(t)>0) and np.all(np.diff(s)>=0)
mask = (s>=t[0]) & (s<=t[-1]); s=s[mask]
assert len(s)>1

def yaw(q):
    return np.unwrap(Rotation.from_quat(np.array(q)[:,[1,2,3,0]]).as_euler('xyz')[:,2])

g = np.array([x['chassis']['xyz_m'] for x in a])
o = np.array([x['xyz_m'] for x in b])[mask]
gy = np.interp(s,t,yaw([x['chassis']['quat_wxyz'] for x in a]))
oy = yaw([x['quat_wxyz'] for x in b])[mask]
g = np.stack([np.interp(s,t,g[:,i]) for i in range(3)],axis=1)
theta = oy[0]-gy[0]
R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
e = np.linalg.norm((g[:,:2]-g[0,:2])@R.T-(o[:,:2]-o[0,:2]),axis=1)
ye = np.rad2deg(gy-gy[0]-oy+oy[0])
print(json.dumps(dict(physics_poses=len(a),ros_odometry_poses=len(b),matched=len(s),matched_sim_time_s=s[[0,-1]].tolist(),sampling_errors=physics[-1]['sampling_errors'],planar_rmse_mm=float(np.sqrt(np.mean(e**2))*1e3),planar_max_mm=float(e.max()*1e3),heading_rmse_deg=float(np.sqrt(np.mean(ye**2)))),indent=2))
