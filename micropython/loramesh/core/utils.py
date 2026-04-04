try:
    import utime as _time
except ImportError:
    import time as _time


def ticks_ms():
    if hasattr(_time, "ticks_ms"):
        return _time.ticks_ms()
    return int(_time.time() * 1000)


def ticks_diff(now, start):
    if hasattr(_time, "ticks_diff"):
        return _time.ticks_diff(now, start)
    return now - start


def sleep_ms(ms):
    if hasattr(_time, "sleep_ms"):
        _time.sleep_ms(ms)
    else:
        _time.sleep(ms / 1000.0)