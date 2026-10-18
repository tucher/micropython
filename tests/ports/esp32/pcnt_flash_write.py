# Test that pulse counter events during flash writes are delivered, not fatal.
# The pin is counted and driven by PWM at once: PWM leaves the pad's input on.

import esp32
import os
from machine import Pin, PWM

events = 0


def on_event(pcnt):
    global events
    pcnt.irq().flags()
    events += 1


pcnt = esp32.PCNT(0, pin=Pin(21), rising=esp32.PCNT.INCREMENT, max=50)
pcnt.irq(on_event, esp32.PCNT.IRQ_MAX)
pcnt.start()
# 20 kHz over 50 counts: 400 events a second while the flash is written.
pwm = PWM(Pin(21), freq=20000, duty_u16=32768)
data = bytes(4096)
for _ in range(50):
    with open("pcnt_flash_write.bin", "wb") as f:
        f.write(data)
    os.remove("pcnt_flash_write.bin")
pwm.deinit()
pcnt.deinit()
print("events delivered:", events > 0)
