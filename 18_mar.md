
t=0s      init published → guard=30s, bg thread starts polling
t=2.5s    close (force/speed/position) arrives → BUFFERED in pending_writes_map_
t=~18s    AG-160 finishes calibration:
            • If FC04 poll sees init_state=2: guard shrinks to now+500ms
            • Otherwise: guard expires naturally at t=30s
t=~18.5s  spin() calls drain_write_commands() → post-loop flush fires
            → replays force/speed/position → AG-160 moves ✓
