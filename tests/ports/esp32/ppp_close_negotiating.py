# Test that a PPP session deactivated while LCP still negotiates is freed:
# lwIP must not keep it, with its timers calling back, and deactivating it
# must not wait for those timers to run out.

import esp32
import gc
import time
from machine import UART

try:
    import network

    network.PPP
except (ImportError, AttributeError):
    print("SKIP")
    raise SystemExit

# Two pins with nothing on them: no peer ever answers.
TX, RX = 21, 22


def idf_free():
    return sum(r[1] for r in esp32.idf_heap_info(esp32.HEAP_DATA))


uart = UART(1, 115200, tx=TX, rx=RX, txbuf=4096)


def session():
    ppp = network.PPP(uart)
    ppp.active(True)
    ppp.connect()
    time.sleep_ms(100)
    t = time.ticks_ms()
    ppp.active(False)
    return time.ticks_diff(time.ticks_ms(), t)


session()  # what the first session allocates for good
gc.collect()
before = idf_free()
slowest = max(session() for _ in range(5))
gc.collect()
print("closed at once:", slowest < 100)
print("freed:", before - idf_free() < 2048)
uart.deinit()
