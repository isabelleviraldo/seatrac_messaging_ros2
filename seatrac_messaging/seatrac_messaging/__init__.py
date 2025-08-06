# Diagnostics
self._skipped_sends = 0
self._data_send_start_time = None
self._last_receive_time = time.time()
self._watchdog_timeout_sec = 10.0

# Start watchdog
self.create_timer(2.0, self._watchdog_timer_cb)
