What changed and why this fixes it:

The core problem was a fundamental mismatch: the AG-160's calibration stroke duration varies by starting position and is not reliably predictable by a wall-clock timer. 7500ms, 5500ms, even 10000ms are all guesses. The QNC Python test that works does not use a timer — it polls init_state until it reads 2.

The bridge now does the same:


init received → send FC06 to 0x0100 → start polling FC04 at 0x0200 every 300ms    init_state=1? → keep waiting    init_state=2? → "✓ init_state=2 confirmed" → set 500ms flush guard → exit poll    timeout (12s)?  → ⚠ keep large guard as fallback
Once the bridge logs ✓ init_state=2 confirmed, the hardware is definitely ready. The 500ms residual guard then flushes any DDS queue backlog. Because the robot uses --delay 8000, the close writes arrive ~1.5–7+ seconds after the guard clears — well within the window.
