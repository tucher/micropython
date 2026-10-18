# Test Pin.pull(), which reads the pull back from the pad.

from machine import Pin

p = Pin(21, Pin.IN, Pin.PULL_UP)
print(p.pull() == Pin.PULL_UP)
p.pull(Pin.PULL_DOWN)
print(p.pull() == Pin.PULL_DOWN)
p.pull(None)
print(p.pull())
p.init(pull=Pin.PULL_UP)
print(p.pull() == Pin.PULL_UP)
p.init(pull=None)
