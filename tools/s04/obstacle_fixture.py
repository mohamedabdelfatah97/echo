"""Opt-in S04 simulation fixture. No ROS command publisher and no USD file saves."""
import json
import math
from pathlib import Path
import time

from pxr import Gf, Usd, UsdGeom, UsdPhysics

CHASSIS = '/World/Nova_Carter_ROS/chassis_link'
OBSTACLE = '/ECHO_S04_TestObstacle'
# Historical audited chassis-frame collider envelope (NAV-0 report 20261123).
X_LIMITS = (-0.587, 0.120)
Y_LIMITS = (-0.238, 0.238)


class ObstacleFixture:
    def __init__(self, stage, timeline, directory):
        self.timeline = timeline
        self.directory = Path(directory).expanduser().resolve(strict=True)
        if not self.directory.is_dir():
            raise RuntimeError('Fixture output must be an existing directory')
        for name in ('fixture.json', 'physics.jsonl'):
            if (self.directory / name).exists():
                raise RuntimeError(f'Refusing to overwrite {name}')
        if timeline.is_playing():
            raise RuntimeError('Fixture must be created before Play')
        if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-8 or UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
            raise RuntimeError('Fixture requires a metre-scale, Z-up stage')
        prim = stage.GetPrimAtPath(CHASSIS)
        if not prim.IsValid() or stage.GetPrimAtPath(OBSTACLE).IsValid():
            raise RuntimeError('Unexpected chassis or fixture prim state')
        matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
        origin = matrix.ExtractTranslation()
        forward = matrix.TransformDir(Gf.Vec3d(1, 0, 0))
        yaw = math.atan2(forward[1], forward[0])
        self.axis = (math.cos(yaw), math.sin(yaw))
        self.origin = (float(origin[0]), float(origin[1]))
        self.near = 0.60
        center = Gf.Vec3d(origin[0] + 0.70*self.axis[0], origin[1] + 0.70*self.axis[1], origin[2] + 0.50)
        # Session layer only: no modifications to the authored scene on disk.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            cube = UsdGeom.Cube.Define(stage, OBSTACLE)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(center)
            cube.AddRotateZOp().Set(math.degrees(yaw))
            cube.AddScaleOp().Set(Gf.Vec3f(0.20, 0.60, 0.90))
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.38, 0.05)])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        self.meta = {
            'obstacle_prim': OBSTACLE, 'chassis_prim': CHASSIS,
            'stage_root': str(stage.GetRootLayer().identifier),
            'obstacle_center_world_m': list(center), 'obstacle_dimensions_m': [0.20, 0.60, 0.90],
            'initial_chassis_world_m': list(origin), 'initial_chassis_yaw_rad': yaw,
            'forward_world_xy': self.axis, 'near_face_initial_forward_m': self.near,
            'historical_collider_x_limits_m': X_LIMITS, 'historical_collider_y_limits_m': Y_LIMITS,
            'nominal_initial_forward_clearance_m': self.near - X_LIMITS[1],
            'note': 'Static session-layer collider; clearance is projected along the initial forward axis using the previously audited chassis-frame XY envelope, not a contact sensor. No stage saves.',
        }
        with (self.directory / 'fixture.json').open('x') as out:
            json.dump(self.meta, out, indent=2)
        self.handle = (self.directory / 'physics.jsonl').open('x', buffering=1)
        self.chassis = None
        self.last = None
        self.failed = False
        self.handle.write(json.dumps({'type': 'metadata', **self.meta}) + '\n')
        print('[ECHO S04] Static obstacle installed:', OBSTACLE, 'output:', self.directory, flush=True)

    def sample(self):
        if self.failed or not self.timeline.is_playing():
            return
        try:
            from isaacsim.core.simulation_manager import SimulationManager
            t = float(SimulationManager.get_simulation_time())
            if self.last is not None and t < self.last:
                raise RuntimeError('Simulation time reset; start a fresh test')
            if self.last is not None and t-self.last < 0.099:
                return
            if self.chassis is None:
                from isaacsim.core.experimental.prims import RigidPrim
                self.chassis = RigidPrim(CHASSIS)
            xyz, quat = self.chassis.get_world_poses()
            import numpy as np
            p = np.asarray(xyz.numpy() if hasattr(xyz, 'numpy') else xyz, dtype=float)[0]
            q = np.asarray(quat.numpy() if hasattr(quat, 'numpy') else quat, dtype=float)[0]
            w, x, y, z = q
            # World XY components of rotated local chassis X and Y unit vectors.
            vx = (1-2*(y*y+z*z), 2*(x*y+w*z))
            vy = (2*(x*y-w*z), 1-2*(x*x+z*z))
            a, b = self.axis
            support = max(a*(p[0]-self.origin[0]+u*vx[0]+v*vy[0]) + b*(p[1]-self.origin[1]+u*vx[1]+v*vy[1]) for u in X_LIMITS for v in Y_LIMITS)
            self.handle.write(json.dumps({'type':'pose','sim_time_s': t, 'timeline_time_s': float(self.timeline.get_current_time()), 'wall_utc_ns': time.time_ns(), 'xyz_m': p.tolist(), 'quat_wxyz': q.tolist(), 'projected_clearance_m': self.near-float(support)}, allow_nan=False)+'\n')
            self.last = t
        except Exception as exc:
            self.failed = True
            self.handle.write(json.dumps({'type': 'error', 'error': repr(exc)})+'\n')
            print('[ECHO S04] Fixture pose logger failed; do not execute test:', repr(exc), flush=True)

    def close(self):
        self.handle.close()
