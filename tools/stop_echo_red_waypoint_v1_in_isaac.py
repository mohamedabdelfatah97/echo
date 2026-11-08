"""Stop ECHO M03C waypoint motion; run in Isaac Sim Script Editor."""
import builtins
job = getattr(builtins, "_echo_red_waypoint_v1", None)
if job is not None and not job.done():
    job.cancel()
    print("ECHO M03C cancellation requested; initial pose restored on next app update")
else:
    print("ECHO M03C not running")
