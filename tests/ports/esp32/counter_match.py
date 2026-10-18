# Test machine.Counter's match and IRQ_MATCH. The pin is counted and driven
# by PWM at once: PWM leaves the pad's input on.

import time
from machine import Counter, Pin, PWM

calls = []


def handler(counter):
    calls.append((counter.irq().flags(), counter.value()))


counter = Counter(0, Pin(21), edge=Counter.RISING, match=500)
counter.irq(handler, Counter.IRQ_MATCH)
pwm = PWM(Pin(21), freq=5000, duty_u16=32768)
time.sleep_ms(300)
pwm.deinit()
total = counter.value()
counter.deinit()
print("called once:", len(calls) == 1)
print("for the match:", calls[0][0] == Counter.IRQ_MATCH)
print("at or after 500:", calls[0][1] >= 500)
print("counting went on:", total > 500)

try:
    Counter(0, Pin(21), match=500).irq(handler, Counter.IRQ_MATCH, hard=True)
except ValueError:
    print("hard=True: ValueError")
Counter(0).deinit()
