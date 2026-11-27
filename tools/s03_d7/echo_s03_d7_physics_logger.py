#!/usr/bin/env python3
"""Passive Isaac Sim in-process logger, adapted from the validated D4F pose trace."""
import builtins
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdGeom

ROOT = '/World/Nova_Carter_ROS/chassis_link'
SIDES = ('front', 'rear', 'left', 'right')
CAMERAS = {
    s: f'{ROOT}/sensors/echo_d455_{s}/RSD455/Camera_Pseudo_Depth'
    for s in SIDES
}
DURATION_SIM_SECONDS = 180.0
MAX_WALL_SECONDS = 420.0
MAX_RECORDS = 24000


def vec(value):
    if hasattr(value, 'numpy'):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64).reshape(-1).tolist()


def get_pose(wrapper):
    xyz, quat = wrapper.get_world_poses()
    # Match the successfully tested standalone D4B/D4D shape handling:
    # Warp-backed pose arrays must be converted to NumPy before indexing.
    return {
        'xyz_m': np.asarray(xyz.numpy() if hasattr(xyz, 'numpy') else xyz,
                            dtype=np.float64)[0].tolist(),
        'quat_wxyz': np.asarray(quat.numpy() if hasattr(quat, 'numpy') else quat,
                                dtype=np.float64)[0].tolist(),
    }


def graph_evidence(stage):
    rows = []
    for prim in stage.Traverse():
        kind = str(prim.GetTypeName())
        attr = prim.GetAttribute('node:type')
        node_type = str(attr.Get()) if attr and attr.IsValid() else ''
        path = str(prim.GetPath())
        if not any(x in (path + ' ' + node_type).lower() for x in
                   ('odometry', 'publishclock', 'readsimulationtime', 'ros_clock', 'computeodom')):
            continue
        row = {'path': path, 'type': kind, 'node_type': node_type, 'attributes': {}, 'relationships': {}}
        for a in prim.GetAttributes():
            name = a.GetName()
            if name.startswith(('inputs:', 'outputs:')) or name == 'node:type':
                try:
                    row['attributes'][name] = {'value': str(a.Get()), 'connections': [str(c) for c in a.GetConnections()]}
                except Exception as exc:
                    row['attributes'][name] = {'error': str(exc)}
        for r in prim.GetRelationships():
            row['relationships'][r.GetName()] = [str(t) for t in r.GetTargets()]
        rows.append(row)
    return rows


class PoseLogger:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError('No active Isaac Sim stage; start the ECHO supervisor')
        for path in [ROOT, *CAMERAS.values()]:
            if not self.stage.GetPrimAtPath(path).IsValid():
                raise RuntimeError(f'Required ECHO prim not present: {path}')
        if not self.timeline.is_playing():
            raise RuntimeError('Press Play in the existing ECHO scene before starting the logger')

        self.chassis = RigidPrim(ROOT)
        self.cameras = {s: XformPrim(path) for s, path in CAMERAS.items()}
        output_dir = Path.home() / 'Downloads' / 'echo_s03_d7_trial2'
        output_dir.mkdir(parents=True, exist_ok=True)
        self.path = output_dir / 'physics.jsonl'
        self.wall_start = time.monotonic()
        self.handle = self.path.open('x', encoding='utf-8')
        self.count = 0
        self.started_sim_time = None
        self.last_time_key = None
        self.subscription = None
        self.finished = False
        self.exception_count = 0
        self.handle.write(json.dumps({
            'type': 'metadata',
            'purpose': 'D7 independent physics chassis trajectory concurrent with ROS odometry',
            'graph_evidence': graph_evidence(self.stage),
            'stage_root_layer': str(self.stage.GetRootLayer().identifier),
            'meters_per_unit': float(UsdGeom.GetStageMetersPerUnit(self.stage)),
            'up_axis': str(UsdGeom.GetStageUpAxis(self.stage)),
            'chassis_prim': ROOT,
            'depth_camera_prims': CAMERAS,
            'quat_order': 'wxyz',
            'duration_sim_seconds': DURATION_SIM_SECONDS,
            'pose_api': 'RigidPrim.get_world_poses / XformPrim.get_world_poses',
            'time_domains': ['SimulationManager.get_simulation_time',
                             'timeline.get_current_time'],
            'note': 'ROS/header equality and renderer timing not yet verified',
        }) + '\n')
        self.handle.flush()
        self.subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self.on_update
        )
        print('[ECHO D7] Passive pose trace STARTED:', self.path, flush=True)
        print('[ECHO D7] Keep Play active; start the D7 ROS logger and supervised goal.', flush=True)
        print('[ECHO D7] Auto-stops after 180 s simulation time or 420 s wall time. Terminal stop request supported.', flush=True)

    def on_update(self, event):
        if self.finished:
            return
        if time.monotonic() - self.wall_start >= MAX_WALL_SECONDS:
            self.stop()
            return
        if not self.timeline.is_playing():
            return
        try:
            timeline_t = float(self.timeline.get_current_time())
            try:
                sim_t = float(SimulationManager.get_simulation_time())
            except Exception:
                sim_t = None
            key = (timeline_t, sim_t)
            if key == self.last_time_key:
                return
            self.last_time_key = key
            if self.started_sim_time is None:
                self.started_sim_time = sim_t if sim_t is not None else timeline_t
            row = {
                'type': 'pose',
                'timeline_time_s': timeline_t,
                'simulation_manager_time_s': sim_t,
                'wall_monotonic_s': time.monotonic(),
                'wall_utc_ns': time.time_ns(),
                'chassis': get_pose(self.chassis),
                'cameras': {s: get_pose(w) for s, w in self.cameras.items()},
            }
            self.handle.write(json.dumps(row, allow_nan=False) + '\n')
            self.count += 1
            if self.count % 20 == 0:
                self.handle.flush()
            if self.count == 1 or self.count % 600 == 0:
                print('[ECHO D7] poses:', self.count,
                      'sim_time:', sim_t, 'timeline:', timeline_t, flush=True)
            effective_t = sim_t if sim_t is not None else timeline_t
            if ((effective_t - self.started_sim_time >= DURATION_SIM_SECONDS)
                    or self.count >= MAX_RECORDS):
                self.stop()
        except Exception as exc:
            self.exception_count += 1
            print('[ECHO D7] Pose sample error:', repr(exc), flush=True)
            if self.exception_count >= 3:
                self.stop()

    def stop(self):
        if self.finished:
            print('[ECHO D7] Already stopped:', self.path)
            return
        self.finished = True
        self.subscription = None  # release callback subscription
        self.handle.write(json.dumps({'type': 'summary', 'records': self.count, 'sampling_errors': self.exception_count}) + '\n')
        self.handle.flush()
        self.handle.close()
        print('[ECHO D7] Trace SAVED:', self.path, flush=True)
        print('[ECHO D7] Records:', self.count,
              '| sampling errors:', self.exception_count, flush=True)


prior = getattr(builtins, '_echo_s03_d7_pose_logger', None)
if prior is not None and not prior.finished:
    raise RuntimeError('D7 logger is already active. Stop it before starting another.')
builtins._echo_s03_d7_pose_logger = PoseLogger()
