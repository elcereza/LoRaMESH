try:
    import utime as _time
except ImportError:
    import time as _time

from .ITickSource import ITickSource


class MaixTicks(ITickSource):
    def millis(self):
        if hasattr(_time, "ticks_ms"):
            return _time.ticks_ms()
        return int(_time.time() * 1000)