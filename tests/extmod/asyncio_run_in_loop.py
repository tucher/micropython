# Test that a loop cannot be started from inside a running one.

try:
    import asyncio
except ImportError:
    print("SKIP")
    raise SystemExit


async def inner():
    return 1


async def main():
    coro = inner()
    try:
        asyncio.run(coro)
    except RuntimeError:
        print("asyncio.run: RuntimeError")
    coro.close()
    coro = inner()
    try:
        asyncio.get_event_loop().run_until_complete(coro)
    except RuntimeError:
        print("run_until_complete: RuntimeError")
    coro.close()
    await asyncio.sleep(0)
    return 2


print(asyncio.run(main()))
# The loop is whole afterwards.
print(asyncio.run(inner()))


# A loop left by an exception the scheduler does not catch is not running.
async def interrupted():
    raise KeyboardInterrupt


try:
    asyncio.run(interrupted())
except KeyboardInterrupt:
    print("KeyboardInterrupt")
print(asyncio.run(inner()))
