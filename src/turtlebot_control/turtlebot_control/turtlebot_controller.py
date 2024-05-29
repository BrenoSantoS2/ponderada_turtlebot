import sys
import os
import asyncio
import threading
from tkinter import Tk, Button, Label, Frame
from PIL import Image, ImageTk
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import websockets
import cv2
import numpy as np
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lin_speed = 0.0
        self.ang_speed = 0.0

    def update_velocity(self):
        msg = Twist()
        msg.linear.x = self.lin_speed
        msg.angular.z = self.ang_speed
        self.publisher_.publish(msg)

    def stop(self):
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        self.update_velocity()

    def increase_lin_speed(self):
        self.lin_speed += 0.1

    def decrease_lin_speed(self):
        self.lin_speed -= 0.1

    def increase_ang_speed(self):
        self.ang_speed += 0.1

    def decrease_ang_speed(self):
        self.ang_speed -= 0.1

class MainWindow:
    def __init__(self, turtlebot_controller):
        self.turtlebot_controller = turtlebot_controller
        self.root = Tk()
        self.root.title("TurtleBot 3 Controller")

        self.frame = Frame(self.root)
        self.frame.pack()

        self.btn_increase_lin = Button(self.frame, text="Increase Linear Speed", command=self.increase_lin_speed)
        self.btn_increase_lin.pack()

        self.btn_decrease_lin = Button(self.frame, text="Decrease Linear Speed", command=self.decrease_lin_speed)
        self.btn_decrease_lin.pack()

        self.btn_increase_ang = Button(self.frame, text="Increase Angular Speed", command=self.increase_ang_speed)
        self.btn_increase_ang.pack()

        self.btn_decrease_ang = Button(self.frame, text="Decrease Angular Speed", command=self.decrease_ang_speed)
        self.btn_decrease_ang.pack()

        self.btn_stop = Button(self.frame, text="Stop", command=self.stop)
        self.btn_stop.pack()

        self.speed_label = Label(self.frame, text="Velocidade linear: 0.0 m/s | Velocidade Angular: 0.0 rad/s")
        self.speed_label.pack()

        self.latency_label = Label(self.frame, text="Latência do Frame: 0.0 segundos")  # Rótulo para exibir a latência
        self.latency_label.pack()

        self.video_label = Label(self.frame, text="Conectando ao vídeo...")
        self.video_label.pack()

        self.root.after(100, self.update_velocity)

        self.video_thread = threading.Thread(target=self.start_video_stream)
        self.video_thread.start()

    def increase_lin_speed(self):
        self.turtlebot_controller.increase_lin_speed()
        self.update_speed_label()

    def decrease_lin_speed(self):
        self.turtlebot_controller.decrease_lin_speed()
        self.update_speed_label()

    def increase_ang_speed(self):
        self.turtlebot_controller.increase_ang_speed()
        self.update_speed_label()

    def decrease_ang_speed(self):
        self.turtlebot_controller.decrease_ang_speed()
        self.update_speed_label()

    def stop(self):
        self.turtlebot_controller.stop()
        self.update_speed_label()

    def update_velocity(self):
        self.turtlebot_controller.update_velocity()
        self.root.after(100, self.update_velocity)

    def update_speed_label(self):
        self.speed_label.config(text=f"Velocidade linear: {self.turtlebot_controller.lin_speed:.2f} m/s | Velocidade Angular: {self.turtlebot_controller.ang_speed:.2f} rad/s")

    async def video_stream(self):
        uri = "ws://localhost:8765"
        async with websockets.connect(uri) as websocket:
            while True:
                frame_start_time = time.time()  # Registra o tempo quando o frame é recebido
                frame = await websocket.recv()
                np_arr = np.frombuffer(frame, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.resize(img, (640, 480))

                img_pil = Image.fromarray(img)
                img_tk = ImageTk.PhotoImage(image=img_pil)

                self.video_label.config(image=img_tk)
                self.video_label.image = img_tk

                latency = time.time() - frame_start_time
                self.latency_label.config(text=f"Latência do Frame: {latency:.2f} segundos")

    def start_video_stream(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.video_stream())

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    turtlebot_controller = TurtleBotController()

    main_window = MainWindow(turtlebot_controller)
    
    executor = threading.Thread(target=rclpy.spin, args=(turtlebot_controller,))
    executor.start()

    main_window.run()

    turtlebot_controller.destroy_node()
    rclpy.shutdown()
    executor.join()

if __name__ == '__main__':
    main()
