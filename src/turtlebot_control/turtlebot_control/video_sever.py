import cv2
import asyncio
import websockets

async def video_stream(websocket, path):
    cap = cv2.VideoCapture(0)  # Substitua com o índice da câmera do TurtleBot
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            _, buffer = cv2.imencode('.jpg', frame)
            await websocket.send(buffer.tobytes())
            await asyncio.sleep(0.1)  # Ajuste conforme necessário
    finally:
        cap.release()

async def main():
    async with websockets.serve(video_stream, "localhost", 8765):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
