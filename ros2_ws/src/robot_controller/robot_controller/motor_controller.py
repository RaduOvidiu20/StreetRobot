import rclpy
from rclpy.node import Node
import tkinter as tk
import serial
import time
import threading
import math
from geometry_msgs.msg import Twist


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.serial_port = serial.Serial("/dev/ttyUSB1", 115200)
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.handle_cmd_vel, 10
        )

    def handle_cmd_vel(self, msg):
        linear_speed = int(msg.linear.x * 255)
        angular_speed = int(msg.angular.z * 255)
        command = f"{linear_speed},{angular_speed}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")

        # Set up the UI
        self.window = tk.Tk()
        self.window.title("Motor Control GUI")

        # Motor 1 Speed Controls
        self.speed1_label = tk.Label(self.window, text="Speed Motor 1")
        self.speed1_label.pack()
        self.speed1_scale = tk.Scale(
            self.window,
            from_=-255,
            to=255,
            orient=tk.HORIZONTAL,
            command=self.send_command,
        )
        self.speed1_scale.pack()

        self.speed1_entry = tk.Entry(self.window, width=5)
        self.speed1_entry.pack()

        self.speed1_display = tk.Label(self.window, text="Direction: Stop | Speed: 0")
        self.speed1_display.pack()

        # Motor 2 Speed Controls
        self.speed2_label = tk.Label(self.window, text="Speed Motor 2")
        self.speed2_label.pack()
        self.speed2_scale = tk.Scale(
            self.window,
            from_=-255,
            to=255,
            orient=tk.HORIZONTAL,
            command=self.send_command,
        )
        self.speed2_scale.pack()

        self.speed2_entry = tk.Entry(self.window, width=5)
        self.speed2_entry.pack()

        self.speed2_display = tk.Label(self.window, text="Direction: Stop | Speed: 0")
        self.speed2_display.pack()

        # Canvas for wheel rotation illustration
        self.canvas = tk.Canvas(self.window, width=200, height=120, bg="white")
        self.canvas.pack()

        # Fixed circles representing the wheels
        self.wheel1 = self.canvas.create_oval(50, 40, 70, 60, outline="green", width=2)
        self.wheel2 = self.canvas.create_oval(
            130, 40, 150, 60, outline="green", width=2
        )

        # Markers that will "rotate" around the wheels based on encoder feedback
        self.wheel1_marker = self.canvas.create_line(
            60, 50, 70, 50, fill="green", width=2
        )
        self.wheel2_marker = self.canvas.create_line(
            140, 50, 150, 50, fill="green", width=2
        )

        # Labels to display encoder values below each wheel
        self.encoder1_label = tk.Label(self.window, text="Encoder 1: 0")
        self.encoder1_label.pack()
        self.encoder2_label = tk.Label(self.window, text="Encoder 2: 0")
        self.encoder2_label.pack()

        # Thread to read encoder feedback in real-time
        self.encoder_data = [0, 0]
        self.running = True
        self.feedback_thread = threading.Thread(target=self.read_encoder_feedback)
        self.feedback_thread.start()

    def send_command(self, *args):
        # Retrieve speed values from entries or fall back to sliders
        speed1 = self.get_speed_from_input(self.speed1_entry, self.speed1_scale)
        speed2 = self.get_speed_from_input(self.speed2_entry, self.speed2_scale)

        # Update the display based on the current speeds
        self.update_display()

        # Build command string for ESP32
        command = f"{speed1},{speed2}\n"

        # Send command via serial in real-time
        if self.serial_port.is_open:
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent command: {command.strip()}")
        else:
            self.get_logger().error("Serial port is not open")

    def get_speed_from_input(self, entry, scale):
        # Check if the entry has a valid integer; if not, fall back to the slider value
        try:
            value = int(entry.get())
            if -255 <= value <= 255:
                return value
            else:
                raise ValueError("Value out of range")
        except ValueError:
            return scale.get()

    def update_display(self):
        # Update display labels based on the slider values
        speed1 = self.speed1_scale.get()
        speed2 = self.speed2_scale.get()

        # Update Motor 1 Display
        direction1 = "Forward" if speed1 > 0 else "Backward" if speed1 < 0 else "Stop"
        self.speed1_display.config(
            text=f"Direction: {direction1} | Speed: {abs(speed1)}"
        )

        # Update Motor 2 Display
        direction2 = "Forward" if speed2 > 0 else "Backward" if speed2 < 0 else "Stop"
        self.speed2_display.config(
            text=f"Direction: {direction2} | Speed: {abs(speed2)}"
        )

    def read_encoder_feedback(self):
        # This function reads encoder feedback from the ESP32 and updates the wheel movement
        while self.running:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode().strip()
                if "Encoder 1 position:" in line and "Encoder 2 position:" in line:
                    try:
                        parts = line.split(", ")
                        self.encoder_data[0] = int(parts[0].split(": ")[1])
                        self.encoder_data[1] = int(parts[1].split(": ")[1])
                        self.update_wheel_rotation()
                        self.update_encoder_display()
                    except (IndexError, ValueError):
                        pass  # Ignore malformed feedback

    def update_wheel_rotation(self):
        # Rotate markers around the fixed circles based on encoder data
        encoder1 = self.encoder_data[0]
        encoder2 = self.encoder_data[1]

        # Calculate the angle for each wheel based on the encoder values
        angle1 = (encoder1 % 360) * math.pi / 180  # Convert to radians
        angle2 = (encoder2 % 360) * math.pi / 180

        # Radius and center points for wheel markers
        radius = 10
        center1 = (60, 50)
        center2 = (140, 50)

        # Calculate new positions for the markers to simulate rotation
        x1 = center1[0] + radius * math.cos(angle1)
        y1 = center1[1] + radius * math.sin(angle1)
        x2 = center2[0] + radius * math.cos(angle2)
        y2 = center2[1] + radius * math.sin(angle2)

        # Update marker positions
        self.canvas.coords(self.wheel1_marker, center1[0], center1[1], x1, y1)
        self.canvas.coords(self.wheel2_marker, center2[0], center2[1], x2, y2)

    def update_encoder_display(self):
        # Update the encoder labels below each wheel with the current encoder values
        self.encoder1_label.config(text=f"Encoder 1: {self.encoder_data[0]}")
        self.encoder2_label.config(text=f"Encoder 2: {self.encoder_data[1]}")

    def run(self):
        # Start the Tkinter UI loop
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.mainloop()

    def on_closing(self):
        # Stop the feedback thread when closing the GUI
        self.running = False
        self.feedback_thread.join()
        self.window.destroy()


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    motor_control_node.run()
    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
