QNC received and processed the latest send.


Bridge is alive:

qnc_bridge PID 25939
One important signal in the log:

[Stats/periodic] tx=10 rx=0 err=6 uptime=83s
So command delivery from robot -> QNC is working, but AG-160 still has read failures on this improved bridge run (err=6, rx=0 at that stats point). That suggests the improved bridge is dispatching writes correctly, but the readback path to the physical AG-160 remains unreliable on this run.

