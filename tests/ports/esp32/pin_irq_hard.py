# Test Pin.irq's hard argument: accepted as False, refused as True, since
# the port runs every pin handler from the scheduler.

from machine import Pin

p = Pin(21, Pin.IN)
p.irq(lambda p: None, Pin.IRQ_RISING, hard=False)
print("hard=False accepted")
try:
    p.irq(lambda p: None, Pin.IRQ_RISING, hard=True)
except ValueError as e:
    print("hard=True:", e)
p.irq(None)
