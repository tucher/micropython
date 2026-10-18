# Test that deinitialising a UART PPP is writing to leaves the board running:
# PPP keeps the UART's C writer, and LCP writes another request a few seconds
# later -- after deinit() took the writer's queue away.

import time
from machine import UART

try:
    import network

    network.PPP
except (ImportError, AttributeError):
    print("SKIP")
    raise SystemExit

# Two pins with nothing on them. Not one for both: LCP would hear its own
# requests, take the link for a loop and stop writing.
TX, RX = 21, 22

# A TX ring big enough for PPP to take the UART's C writer.
uart = UART(1, 115200, tx=TX, rx=RX, txbuf=4096)
ppp = network.PPP(uart)
ppp.active(True)
ppp.connect()
time.sleep_ms(200)
uart.deinit()
time.sleep_ms(8000)  # two of LCP's restarts, 3 s apart
print("running")
ppp.active(False)
