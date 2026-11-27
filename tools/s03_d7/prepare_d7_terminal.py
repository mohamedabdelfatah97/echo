#!/usr/bin/env python3
"""Prepare private D7 launcher copies; never edit ~/echo or USD files."""
from pathlib import Path
import hashlib
import shlex
import subprocess

home = Path.home()
repo = home / 'echo'
kit = Path(__file__).resolve().parent
trial = home / 'Downloads/echo_s03_d7_trial2'
if trial.exists():
    raise SystemExit(f'ERROR: {trial} already exists. Preserve it before preparing another trial.')


def replace_once(text, old, new, label):
    if text.count(old) != 1:
        raise SystemExit(f'ERROR: unexpected {label} source; no repository changes made.')
    return text.replace(old, new, 1)


def read(name):
    p = repo / 'scripts' / name
    data = p.read_bytes()
    sha = hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()
    print(f'Source {p.name}: {sha[:12]}')
    return data.decode(), sha

sim_name = 'start_full_warehouse_two_humans_four_d455.py'
shell_name = 'start_full_warehouse_two_humans_four_d455.sh'
super_name = 'start_echo_dynamic_four_d455.py'
sim, sim_hash = read(sim_name)
shell, shell_hash = read(shell_name)
supervisor, sup_hash = read(super_name)
if sim_hash != '6cb60f8d785b0e19ce4f74ac2faea28f23ddf28c' or shell_hash != 'e1f929cac11574e6a509ae55bc761c39fe9b245c' or not sup_hash.startswith('a63fb58'):
    raise SystemExit('ERROR: launcher source differs from audited versions. Send the three source files; nothing modified.')
logger = kit / 'echo_s03_d7_physics_logger.py'
ros_logger = kit / 'echo_s03_d7_ros_logger.py'
for p in (logger, ros_logger):
    compile(p.read_text(), str(p), 'exec')

# All requests are one-shot and confined to this fresh trial folder.
hook = '''    import builtins as _d7_builtins
    _d7_dir = Path(__D7_DIR__)
    _d7_script = Path(__D7_SCRIPT__)
    print("[ECHO D7] Terminal control armed:", _d7_dir, flush=True)
    while app.is_running():
        app.update()
        for _d7_action in ("play", "start", "stop"):
            _d7_req = _d7_dir / (_d7_action + ".request")
            if not _d7_req.exists():
                continue
            if _d7_action == "start" and not omni.timeline.get_timeline_interface().is_playing():
                continue
            _d7_req.unlink()
            try:
                if _d7_action == "play":
                    omni.timeline.get_timeline_interface().play()
                elif _d7_action == "start":
                    if (_d7_dir / "physics.jsonl").exists():
                        raise RuntimeError("D7 recording already exists; refusing overwrite")
                    exec(compile(_d7_script.read_text(), str(_d7_script), "exec"), {"__name__": "__main__"})
                elif _d7_action == "stop":
                    _d7_rec = getattr(_d7_builtins, "_echo_s03_d7_pose_logger", None)
                    if _d7_rec is None:
                        raise RuntimeError("D7 physics logger not started")
                    if not _d7_rec.finished:
                        _d7_rec.stop()
                print("[ECHO D7] Request completed:", _d7_action, flush=True)
                (_d7_dir / (_d7_action + ".ok")).touch()
            except Exception:
                _d7_error = traceback.format_exc()
                print(_d7_error, flush=True)
                (_d7_dir / (_d7_action + ".error")).write_text(_d7_error)
'''.replace('__D7_DIR__', repr(str(trial))).replace('__D7_SCRIPT__', repr(str(logger)))
sim = replace_once(sim, '    while app.is_running():\n        app.update()\n', hook, 'Isaac update loop')
# Keep original shell setup, including Isaac Python and ROS environment.
shell = replace_once(shell, 'SCRIPT="$HOME/echo/scripts/start_full_warehouse_two_humans_four_d455.py"', 'SCRIPT=' + shlex.quote(str(trial / sim_name)), 'shell launcher')
supervisor = replace_once(supervisor, 'sim = start("sim", ["bash", str(required[0])])', 'sim = start("sim", ["bash", ' + repr(str(trial / shell_name)) + '])', 'supervisor simulator invocation')
for name, text in ((sim_name, sim), (super_name, supervisor)):
    compile(text, name, 'exec')
subprocess.run(['bash','-n'], input=shell, text=True, check=True)
trial.mkdir(parents=True, exist_ok=False)
for name, text in ((sim_name, sim), (shell_name, shell), (super_name, supervisor)):
    (trial / name).write_text(text)
print('Prepared:', trial)
print('Python and shell syntax: PASS; repository unchanged')
