# Test that a pulse counter leaves its pin's pull as the Pin set it.

from machine import Pin, Counter
import esp32

if not hasattr(Pin, "pull"):
    print("SKIP")
    raise SystemExit

p = Pin(21, Pin.IN, Pin.PULL_DOWN)
pcnt = esp32.PCNT(0, pin=p, rising=esp32.PCNT.INCREMENT)
print("PCNT, pulled down:", p.pull() == Pin.PULL_DOWN)
pcnt.deinit()

p.init(pull=None)
counter = Counter(0, p)
print("Counter, no pull:", p.pull())
counter.init(p, edge=Counter.FALLING)
print("Counter.init, no pull:", p.pull())
counter.deinit()
