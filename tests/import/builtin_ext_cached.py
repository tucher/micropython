# Test that an extensible built-in module nothing on sys.path extends is kept
# in sys.modules once imported, so that the next import finds it there.

import sys

try:
    import time
except ImportError:
    print("SKIP")
    raise SystemExit

print("time" in sys.modules)
print(sys.modules["time"] is time)
