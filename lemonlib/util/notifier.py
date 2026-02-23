class Notifier:
    """Periodically calls a callback on a background thread. (implements edu.wpi.first.wpilibj.Notifier)"""

    def __init__(self, callback: Callable[[], None]):
        self._callback = callback
        self._timer: Optional[threading.Timer] = None
        self._lock = threading.Lock()
        self._running = False
        self._period_s: float = 0.020

    def startPeriodic(self, period_s: float) -> None:
        self._period_s = period_s
        self._running = True
        self._schedule()

    def _schedule(self) -> None:
        if not self._running:
            return
        self._timer = threading.Timer(self._period_s, self._run)
        self._timer.daemon = True
        self._timer.start()

    def _run(self) -> None:
        if not self._running:
            return
        try:
            self._callback()
        finally:
            self._schedule()

    def stop(self) -> None:
        self._running = False
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def close(self) -> None:
        self.stop()