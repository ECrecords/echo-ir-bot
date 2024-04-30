import asyncio

async def handle_client(reader, writer, message_queue):
    while True:
        data = await reader.read(100)  # Reads up to 100 bytes
        if not data:
            break  # No more data, close the connection
        message = data.decode()
        addr = writer.get_extra_info('peername')
        await message_queue.put((message, addr))  # Add message to the queue

    writer.close()

async def process_messages(message_queue):
    while True:
        message, addr = await message_queue.get()  # Wait for a message
        print(f"Received '{message}' from {addr}")
        message_queue.task_done()

async def main(host, port):
    message_queue = asyncio.Queue()
    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, message_queue),
        host, port
    )

    # Start a background task to process messages
    task = asyncio.create_task(process_messages(message_queue))

    async with server:
        print(f"Serving on {server.sockets[0].getsockname()}")
        await server.serve_forever()

if __name__ == "__main__":
    asyncio.run(main('0.0.0.0', 8008))
