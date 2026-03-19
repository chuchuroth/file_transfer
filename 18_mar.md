Root cause
When --delay 14000 is used, the sequence is:

t=0: init FC06 sent → guard armed at t+12s, background thread polls FC04
t=11.5s: background thread gives up (never sees init_state=2)
t=12s: guard expires by time
t=14s: close writes (force/speed/position) arrive → pass the guard → reach AG-160
AG-160 is still calibrating (init_state=1, 160mm stroke takes ≥15-20s) → silently ignores position commands
t=14s: DDS queue is now empty — no writes left to retry when init eventually completes
The gripper never moved because it was still mid-calibration when the motion commands arrived, and then they were consumed and lost.

Five changes to qnc_bridge.cpp:
Change	Detail
pending_writes_map_	New member — stores {reg, val} pairs buffered during init guard (protected by init_wait_mutex_)
wire_addr_for()	New helper — reverse-lookup of wire address by bus pointer (for flush path)
Guard-active branch	Instead of goto next_write_sample (discard), pushes {reg, val} to pending_writes_map_[bus] then skips
Guard-expired branch	Drains pending_writes_map_[bus] via FC06, then falls through to the current write
Post-loop flush	At the end of every drain_write_commands() call, checks for naturally-expired guards and replays any buffered writes — this is what fires when AG-160 finishes calibrating at t=18s and no new DDS write is arriving
init_wait_ms1 default	Raised from 12 000 → 30 000 ms so the guard covers the AG-160's full stroke; the background FC04 thread still shortens it as soon as init_state=2 is confirmed
