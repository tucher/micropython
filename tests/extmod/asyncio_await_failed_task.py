# Test awaiting, more than once, a task that failed before anything awaited it.

try:
    import asyncio
except ImportError:
    print("SKIP")
    raise SystemExit


async def fail():
    raise ValueError("failed")


async def main():
    t = asyncio.create_task(fail())
    # Let the task run and fail with nothing awaiting it.
    await asyncio.sleep(0)
    for i in range(3):
        try:
            await t
        except ValueError as e:
            print(i, "ValueError", e)
        # Let the loop handle the failed task in between.
        await asyncio.sleep(0)
    print("done", t.done())


asyncio.run(main())
