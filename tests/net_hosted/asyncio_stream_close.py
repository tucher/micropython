# Test that Stream.close() closes the connection by itself.

import sys

# Only certain platforms can do TCP/IP loopback.
if sys.platform not in ("darwin", "esp32", "linux"):
    print("SKIP")
    raise SystemExit

try:
    import asyncio
except ImportError:
    print("SKIP")
    raise SystemExit

PORT = 8000


async def handler(reader, writer):
    data = b""
    while True:
        more = await reader.read(100)
        if not more:
            break
        data += more
    print("handler got", data, "then EOF")
    writer.close()
    await writer.wait_closed()
    done.set()


async def main():
    global done
    done = asyncio.Event()
    server = await asyncio.start_server(handler, "127.0.0.1", PORT)
    reader, writer = await asyncio.open_connection("127.0.0.1", PORT)
    writer.write(b"hello")
    await writer.drain()
    # close() alone, with nothing buffered: the peer sees EOF.
    writer.close()
    try:
        await asyncio.wait_for(done.wait(), 2)
    except asyncio.TimeoutError:
        print("no EOF after close()")
    server.close()
    await server.wait_closed()


asyncio.run(main())
