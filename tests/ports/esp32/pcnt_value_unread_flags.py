# Test that PCNT.value() returns when the irq handler leaves the flags unread.
# The pin is counted and driven by PWM at once: PWM leaves the pad's input on.

import esp32
import time
from machine import Pin, PWM

calls = 0


def handler(pcnt):
    # Does not call pcnt.irq().flags().
    global calls
    calls += 1


pcnt = esp32.PCNT(0, pin=Pin(21), rising=esp32.PCNT.INCREMENT, max=50)
pcnt.irq(handler, esp32.PCNT.IRQ_MAX)
pcnt.start()
pwm = PWM(Pin(21), freq=5000, duty_u16=32768)
time.sleep_ms(50)
pwm.deinit()
value = pcnt.value()
pcnt.deinit()
print("value returned:", isinstance(value, int))
print("handler called:", calls > 0)
