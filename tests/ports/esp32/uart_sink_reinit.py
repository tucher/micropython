# Test that a UART's C sink -- PPP's receive path -- still gets the bytes
# after init() reinstalls the driver to change its buffers: the task that
# hands them over must come back with the driver.
#
# TX and RX share one pin, so what is written is received. PPP is active but
# not connecting, so it says nothing itself; bytes the sink takes leave the
# driver's buffer, and any() shows the ones nobody took.

import time
from machine import UART

try:
    import network

    network.PPP
except (ImportError, AttributeError):
    print("SKIP")
    raise SystemExit

PIN = 21


def left_waiting(uart):
    uart.write(b"x" * 20)
    time.sleep_ms(100)
    return uart.any()


uart = UART(1, 115200, tx=PIN, rx=PIN)
ppp = network.PPP(uart)
ppp.active(True)
print("taken:", left_waiting(uart) == 0)
uart.init(rxbuf=1024)
print("taken after init:", left_waiting(uart) == 0)
ppp.active(False)
uart.deinit()
