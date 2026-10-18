# Test that a socket's timeout still ends the call when a callback scheduled
# while it waits runs past the deadline: a blocking call waits in slices and
# runs pending callbacks between them, and the slice after such a callback
# must not wait for ever.

import errno
import micropython
import socket
import time
import _thread

PORT = 34567
TIMEOUT_MS = 50


def wake():
    # Unblocks a call that would otherwise never return.
    time.sleep_ms(1000)
    w = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    w.sendto(b"x", ("127.0.0.1", PORT))
    w.close()


def busy(_):
    # Started from here, not before: a thread running Python while this is
    # pending would run it in place of recv().
    _thread.start_new_thread(wake, ())
    t = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t) < 2 * TIMEOUT_MS:
        pass


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", PORT))
s.settimeout(TIMEOUT_MS / 1000)
t = time.ticks_ms()
try:
    micropython.schedule(busy, None)
    s.recv(8)
    print("recv returned data")
except OSError as e:
    print("timed out:", e.errno == errno.ETIMEDOUT)
print("ended with the callback:", time.ticks_diff(time.ticks_ms(), t) < 500)
time.sleep_ms(1200)
s.close()
