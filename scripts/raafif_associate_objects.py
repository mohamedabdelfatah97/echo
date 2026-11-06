"""RAAFIF Experiment 4: baseline persistent 3D object association.

Uses Experiment 3 observations; strips manually assigned box names prior to
matching. No dependency on SciPy, and no simulator needed.
"""
import argparse
import json
from functools import lru_cache
from math import dist
from pathlib import Path

GATE_M = 0.25
MOVE_THRESHOLD_M = 0.05


def observations(frame):
    # Deliberately do NOT return the manually named left_box/right_box labels.
    return [
        {"xyz_m": value[f"{frame}_m"],
         "visible_pixels": value[f"valid_pixels_{frame}"]}
        for name, value in data.items()
        if name != "ground_truth"
    ]


def associate(tracks, detections):
    # Exact minimum-cost 1:1 matching for a small scene, with missed detections.
    costs = [[dist(t["xyz_m"], d["xyz_m"]) for d in detections]
             for t in tracks]
    missed_cost = GATE_M + 0.01

    @lru_cache(None)
    def solve(i, used):
        if i == len(tracks):
            return 0.0, ()
        cost, pairs = solve(i + 1, used)
        best = (missed_cost + cost, (None,) + pairs)
        for j, candidate_cost in enumerate(costs[i]):
            if used & (1 << j) or candidate_cost > GATE_M:
                continue
            cost, pairs = solve(i + 1, used | (1 << j))
            proposal = (candidate_cost + cost, (j,) + pairs)
            if proposal[0] < best[0]:
                best = proposal
        return best

    return solve(0, 0)[1]


def main():
    global data
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path,
                        default=Path.home()/"raafif/results/experiment_03/movement_results.json")
    parser.add_argument("--output", type=Path,
                        default=Path.home()/"raafif/results/experiment_04/object_memory.json")
    args = parser.parse_args()
    data = json.loads(args.input.read_text())

    first = sorted(observations("before"), key=lambda item: item["xyz_m"][0])
    second = list(reversed(observations("after")))  # shuffle observation order

    memory = []
    for i, obj in enumerate(first, start=1):
        memory.append({
            "id": f"RAAFIF_{i:03d}",
            "xyz_m": obj["xyz_m"],
            "first_seen": "before",
            "last_seen": "before",
            "status": "INITIALIZED",
            "visible_pixels": obj["visible_pixels"],
        })

    assignments = associate(memory, second)
    matched = set()

    for track, j in zip(memory, assignments):
        if j is None:
            track["status"] = "UNOBSERVED"  # NOT automatically removed
            continue
        detected = second[j]
        matched.add(j)
        delta = [b-a for a, b in zip(track["xyz_m"], detected["xyz_m"])]
        displacement = dist(track["xyz_m"], detected["xyz_m"])
        track.update({
            "xyz_m": detected["xyz_m"],
            "last_seen": "after",
            "status": "MOVED" if displacement >= MOVE_THRESHOLD_M else "STATIONARY",
            "delta_xyz_m": delta,
            "displacement_m": displacement,
            "visible_pixels": detected["visible_pixels"],
        })

    for j, detected in enumerate(second):
        if j not in matched:
            memory.append({
                "id": f"RAAFIF_{len(memory)+1:03d}",
                "xyz_m": detected["xyz_m"],
                "first_seen": "after",
                "last_seen": "after",
                "status": "NEW",
                "visible_pixels": detected["visible_pixels"],
            })

    payload = {
        "method": "minimum-total-3d-center-distance",
        "note": "Masks were manually prompted; cross-frame association uses 3D positions only.",
        "association_gate_m": GATE_M,
        "movement_threshold_m": MOVE_THRESHOLD_M,
        "tracks": memory,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2))

    print("=== RAAFIF EXPERIMENT 4: PERSISTENT OBJECT MEMORY ===")
    for track in memory:
        d = track.get("displacement_m")
        suffix = f" | displacement={100*d:.2f} cm" if d is not None else ""
        print(f"{track['id']}: {track['status']}{suffix}")
    print("Saved:", args.output)


if __name__ == "__main__":
    main()
