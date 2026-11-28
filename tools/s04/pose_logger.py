"""Passive chassis pose trace; no scene edits or ROS commands."""
import json
import os
from pathlib import Path
import time
from pxr import UsdGeom

CHASSIS = '/World/Nova_Carter_ROS/chassis_link'

class PoseLogger:
    def __init__(self, stage, timeline, directory):
        self.timeline = timeline
        self.directory = Path(directory).resolve(strict=True)
        if timeline.is_playing() or not stage.GetPrimAtPath(CHASSIS).IsValid():
            raise RuntimeError('Expected stopped timeline and valid chassis')
        if abs(UsdGeom.GetStageMetersPerUnit(stage)-1.0)>1e-8 or UsdGeom.GetStageUpAxis(stage)!=UsdGeom.Tokens.z:
            raise RuntimeError('Expected metre Z-up stage')
        self.handle = (self.directory/'physics.jsonl').open('x', buffering=1)
        self.handle.write(json.dumps(dict(type='metadata', chassis_prim=CHASSIS, stage_root=str(stage.GetRootLayer().identifier), quat_order='wxyz', target_rate_hz=10))+'\n')
        self.dynamic = os.environ.get("ECHO_S04_AGENTS") == "dynamic"
        self.agents = None
        self.last = None
        self.last_seen_sim = None
        self.actor_wait_start = None
        self.actor_wait_sim = None
        self.chassis = None
        self.failed = False
        print('[ECHO S04] Passive chassis pose logger ready', flush=True)

    def sample(self):
        if self.failed or not self.timeline.is_playing():
            return
        try:
            import numpy as np
            from isaacsim.core.simulation_manager import SimulationManager
            t=float(SimulationManager.get_simulation_time())
            if self.last_seen_sim is not None and t<self.last_seen_sim:
                raise RuntimeError('Simulation time reset; start fresh trial')
            self.last_seen_sim = t
            if self.last is not None and t-self.last<.099:
                return
            if self.chassis is None:
                from isaacsim.core.experimental.prims import RigidPrim
                self.chassis=RigidPrim(CHASSIS)
            xyz,quat=self.chassis.get_world_poses()
            p=np.asarray(xyz.numpy() if hasattr(xyz,'numpy') else xyz,dtype=float)[0]
            q=np.asarray(quat.numpy() if hasattr(quat,'numpy') else quat,dtype=float)[0]
            row = dict(type='pose', sim_time_s=t, timeline_time_s=float(self.timeline.get_current_time()), wall_utc_ns=time.time_ns(), xyz_m=p.tolist(), quat_wxyz=q.tolist())
            if self.dynamic:
                # Same runtime API used by the validated M04C IRA logger.
                from omni.metropolis.pipeline.agent import AgentsManager
                agents = list(AgentsManager.get_instance().get_runtime_agent_instances())
                identities = sorted((str(a.name), str(a.prim_path), type(a).__name__) for a in agents)
                # Runtime instances are registered asynchronously after Play.
                # Never publish incomplete actor poses or relax checks after readiness.
                if self.agents is None and len(identities) < 4:
                    now = time.monotonic()
                    if self.actor_wait_start is None:
                        self.actor_wait_start = now
                        self.actor_wait_sim = t
                        print('[ECHO S04] Waiting for four IRA runtime actors', flush=True)
                    if now-self.actor_wait_start >= 15.0 or t-self.actor_wait_sim >= 8.0:
                        raise RuntimeError(f'IRA initialization timed out: {identities}')
                    return
                if len(identities) != 4 or len({i[0] for i in identities}) != 4 or any(not i[0] or not i[1] for i in identities):
                    raise RuntimeError(f'Expected four unique IRA actors: {identities}')
                if self.agents is None:
                    self.agents = identities
                    print('[ECHO S04] IRA identity audit:', identities, flush=True)
                elif self.agents != identities:
                    raise RuntimeError('IRA actor identities changed within trial')
                row['actors'] = []
                for a in sorted(agents, key=lambda a: str(a.name)):
                    pos = a.get_world_position()
                    row['actors'].append(dict(id=str(a.name), prim_path=str(a.prim_path), kind=type(a).__name__, xyz_m=[float(pos[i]) for i in range(3)]))
            self.handle.write(json.dumps(row,allow_nan=False)+'\n')
            self.last=t
        except Exception as exc:
            self.failed=True
            self.handle.write(json.dumps(dict(type='error',error=repr(exc)))+'\n')
            print('[ECHO S04] Pose logger failed:',repr(exc),flush=True)

    def close(self):
        self.handle.close()
