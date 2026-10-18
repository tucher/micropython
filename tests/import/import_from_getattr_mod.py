# Helper for import_from_getattr.py: a module with __getattr__ (PEP 562).


def __getattr__(attr):
    if attr == "present":
        return 1
    raise AttributeError(attr)
