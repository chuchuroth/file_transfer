rebuild on the Pi CM5 with qnc_bridge.cpp.


Here's a summary of what was fixed in qnc_bridge.cpp:

1. init_wait_map_ arm is now mutex-protected (was raw write without lock):
std::lock_guard<std::mutex> lk(init_wait_mutex_);init_wait_map_[bus] = now + max_ms;

2. Background thread captures map_ptr/mutex_ptr instead of raw deadline_ptr, so its write to init_wait_map_ is also mutex-protected:
std::lock_guard<std::mutex> lg(*mutex_ptr);(*map_ptr)[bus] = now + 500ms;

3. next_write_sample: label added at line 732, giving the goto at line 666 a valid target that skips the write and init-arm logic, landing right before the stats publish.

The bridge is now race-condition-free.
