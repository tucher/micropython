# Test "from module import name" when the module's __getattr__ raises
# AttributeError for that name: the import raises ImportError.

import import_from_getattr_mod

if not hasattr(import_from_getattr_mod, "present"):
    print("SKIP")
    raise SystemExit

from import_from_getattr_mod import present

print(present)

try:
    from import_from_getattr_mod import absent
except ImportError:
    print("ImportError")
