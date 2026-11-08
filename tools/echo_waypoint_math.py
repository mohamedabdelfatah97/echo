"""Pure Python geometry and timing planner for ECHO visual-only waypoint probes.

No Isaac Sim imports. A waypoint is a fixed, known world (x,y) plus a dwell.
The caller supplies the actor's actual starting pose from USD.
"""
import math


def smoothstep(t):
    q = max(0.0, min(1.0, float(t)))
    return q * q * (3 - 2 * q)


def shortest_turn_deg(start, target):
    return (target - start + 180.0) % 360.0 - 180.0


def finite(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def build_route(config, start):
    """Return time-indexed segments and total duration without touching USD.

    segment: kind, start_xyz, end_xyz, start_yaw, end_yaw, duration_s.
    Each motion ramp uses smoothstep; peak velocity is 1.5 * delta / duration.
    """
    route = config.get("waypoints")
    if not isinstance(route, list) or not 1 <= len(route) <= 8:
        raise ValueError("waypoints must be a list of 1..8 destinations")
    max_speed = finite(config.get("max_linear_speed_mps"), "max_linear_speed_mps")
    max_turn = finite(config.get("max_angular_speed_degps"), "max_angular_speed_degps")
    max_leg = finite(config.get("max_leg_m"), "max_leg_m")
    max_total = finite(config.get("max_total_distance_m"), "max_total_distance_m")
    if not (0.01 <= max_speed <= 0.15 and 3 <= max_turn <= 30 and
            0.02 <= max_leg <= 1 and 0.02 <= max_total <= 4):
        raise ValueError("Speed/length limits outside conservative pilot bounds")
    mode = config.get("heading_mode")
    if mode not in ("hold_initial", "face_motion"):
        raise ValueError("heading_mode must be 'hold_initial' or 'face_motion'")
    initial = tuple(finite(v, f"start[{i}]") for i, v in enumerate(start))
    if len(initial) != 4:
        raise ValueError("start must be [x,y,z,yaw_deg]")
    xyz = initial[:3]
    yaw = initial[3]
    path_length = 0.0
    segments = []
    for index, wp in enumerate(route):
        if not isinstance(wp, dict):
            raise ValueError(f"waypoint {index} must be an object")
        target_xyz = (finite(wp.get("x"), f"waypoint[{index}].x"),
                      finite(wp.get("y"), f"waypoint[{index}].y"), xyz[2])
        pause = finite(wp.get("pause_s", 0.0), f"waypoint[{index}].pause_s")
        if not 0 <= pause <= 8:
            raise ValueError(f"waypoint {index} pause must be within 0..8 s")
        dx, dy = target_xyz[0]-xyz[0], target_xyz[1]-xyz[1]
        dist = math.hypot(dx, dy)
        if dist > max_leg + 1e-9:
            raise ValueError(f"waypoint {index} leg {dist:.3f}m exceeds max_leg_m={max_leg}")
        path_length += dist
        if path_length > max_total + 1e-9:
            raise ValueError(f"route length {path_length:.3f}m exceeds {max_total}m")
        if dist > 1e-5 and mode == "face_motion":
            target_yaw = math.degrees(math.atan2(dy, dx))
            rotation = shortest_turn_deg(yaw, target_yaw)
            if abs(rotation) > 1e-4:
                duration = max(0.4, 1.5*abs(rotation)/max_turn)
                segments.append(dict(kind="rotate", from_xyz=xyz, to_xyz=xyz,
                                     from_yaw=yaw, to_yaw=yaw+rotation,
                                     duration_s=duration, waypoint=index))
                yaw = yaw + rotation
        if dist > 1e-5:
            duration = max(0.4, 1.5*dist/max_speed)
            segments.append(dict(kind="translate", from_xyz=xyz, to_xyz=target_xyz,
                                 from_yaw=yaw, to_yaw=yaw,
                                 duration_s=duration, waypoint=index))
        xyz = target_xyz
        if pause > 0:
            segments.append(dict(kind="pause", from_xyz=xyz, to_xyz=xyz,
                                 from_yaw=yaw, to_yaw=yaw,
                                 duration_s=pause, waypoint=index))
    if not segments:
        raise ValueError("route has no motion and no pause")
    expected = (initial[0], initial[1], initial[2])
    if bool(config.get("require_return_to_start", True)) and math.dist(xyz, expected) > 0.001:
        raise ValueError("This safety pilot requires last waypoint to return to start")
    if bool(config.get("require_return_to_start", True)):
        reset_turn = shortest_turn_deg(yaw, initial[3])
        if abs(reset_turn) > 1e-4:
            segments.append(dict(kind="restore_heading", from_xyz=xyz, to_xyz=xyz,
                                 from_yaw=yaw, to_yaw=yaw+reset_turn,
                                 duration_s=max(0.4, 1.5*abs(reset_turn)/max_turn),
                                 waypoint=len(route)))
    total = sum(s["duration_s"] for s in segments)
    if total > 120:
        raise ValueError("planned route exceeds 120 simulation seconds")
    return segments, total, path_length
