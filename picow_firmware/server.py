import asyncio

class EchoServerProtocol:
    def __init__(self, message_queue):
        self.message_queue = message_queue

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        # Handle data as binary
        distance = int.from_bytes(data[4:8][::-1], 'big')
        angle = int.from_bytes(data[0:4][::-1], 'big')

        # Handle data as string
        message = f"Distance: {distance} Angle: {angle}"
        asyncio.create_task(self.message_queue.put((message, addr)))

async def process_messages(message_queue):
    while True:
        message, addr = await message_queue.get()  # Wait for a message
        print(f"Received '{message}' from {addr}")
        message_queue.task_done()

async def main(host, port):
    message_queue = asyncio.Queue()

    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: EchoServerProtocol(message_queue),
        local_addr=(host, port)
    )

    # Start a background task to process messages
    task = asyncio.create_task(process_messages(message_queue))

    print(f"Serving on {host}:{port}")
    while True: await asyncio.sleep(1)  # Keep the server running

if __name__ == "__main__":
    asyncio.run(main('0.0.0.0', 8008))