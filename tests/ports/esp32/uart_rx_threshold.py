# Test that UART.init() puts the RX FIFO threshold back when timeout_char
# rises again: a timeout_char shorter than one character lowers it to a
# single byte, an interrupt per byte, and a later, longer one must restore it.
#
# TX and RX share one pin, so the UART hears itself without a jumper. Bytes
# reach the driver's buffer -- and any() -- only when the FIFO holds the
# threshold or the line has gone quiet for timeout_char: halfway through a
# burst, any() is 0 with the default threshold and half the burst with one.

import time
from machine import UART

PIN = 21
BAUD = 9600
BURST = 50  # 52 ms at 9600 baud, fewer bytes than the default threshold


def heard_halfway(uart):
    uart.write(b"x" * BURST)
    time.sleep_ms(26)
    n = uart.any()
    time.sleep_ms(60)  # the rest of the burst, and the line quiet after it
    uart.read()
    return n > 0


uart = UART(1, BAUD, tx=PIN, rx=PIN, timeout_char=20)
print("timeout_char=20:", heard_halfway(uart))
uart.init(timeout_char=0)
print("timeout_char=0:", heard_halfway(uart))
uart.init(timeout_char=20)
print("timeout_char=20 again:", heard_halfway(uart))
uart.deinit()
