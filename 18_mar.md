
The issue was that bridge port1 was pointed at the wrong adapter path for AG160.

If you want, I can make this persistent next by updating the udev serial-to-symlink mapping (or your bridge startup env) so reboots keep the correct AG160 port without manual overrides.
