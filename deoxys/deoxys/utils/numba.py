"""
Numba utils.
"""
try:
    import numba

    ENABLE_NUMBA = True
    CACHE_NUMBA = True
except:
    ENABLE_NUMBA = False
    CACHE_NUMBA = False


def jit_decorator(func):
    if ENABLE_NUMBA:
        return numba.jit(nopython=True, cache=CACHE_NUMBA)(func)
    return func
