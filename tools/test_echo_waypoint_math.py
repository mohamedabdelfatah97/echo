"""Run from normal Ubuntu Python: python3 ~/echo/tools/test_echo_waypoint_math.py"""
import math
from echo_waypoint_math import build_route, smoothstep, shortest_turn_deg

cfg = {
    "heading_mode":"hold_initial", "max_linear_speed_mps":0.08,
    "max_angular_speed_degps":15, "max_leg_m":0.25,
    "max_total_distance_m":0.5, "require_return_to_start":True,
    "waypoints":[{"x":2, "y":-2.8, "pause_s":1},
                 {"x":2,"y":-3,"pause_s":0}],
}
segments, total, distance = build_route(cfg, (2,-3,0,0))
assert abs(distance - 0.4) < 1e-9
assert abs(total - 8.5) < 1e-6
assert len(segments) == 3
assert smoothstep(0) == 0 and smoothstep(1) == 1
assert abs(shortest_turn_deg(170, -170)-20) < 1e-9
cfg["heading_mode"] = "face_motion"
segments, total, distance = build_route(cfg, (2,-3,0,0))
assert len([s for s in segments if s["kind"] == "rotate"]) == 2
assert len([s for s in segments if s["kind"] == "restore_heading"]) == 1
assert abs(segments[-1]["to_yaw"] % 360) < 1e-9
assert total > 8.5
print("PASS: waypoint geometry and timing smoke tests")
