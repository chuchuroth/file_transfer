
Result from the current run: no ✓ init_state=2 confirmed in the log.

What QNC logged instead:

[init] ... calibration started — polling FC04 init_state (timeout 12000 ms)...
[init] ⚠ ... init_state=2 not confirmed — keeping 12000 ms guard
then force/speed/position writes were skipped while guard active
later:
[init] ... calibration complete, ready for motion
So in this run, the bridge did not observe init_state==2 via its FC04 polling path, and fell back to the time guard behavior.

This suggests your current AG160 status read path at init time is not reliably returning init_state=2 via FC04 on QNC, even though physical movement proceeds and later motion works.

