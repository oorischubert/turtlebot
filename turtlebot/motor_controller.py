import rclpy
from turtlebot.motorComms import MotorController, MotorMessage
from turtlebot.configuration import *
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
import sys
import time

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        # esp comms init 
        self.esp = MotorController(ESP_SERIAL_PORT)
        self.motorMessage = MotorMessage()

        connectBool = self.esp.initHandshake()
        if not connectBool: 
            self.get_logger().info("Error opening serial port!")
        else: 
            self.get_logger().info("Serial port opened successfully!")
            
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10  # Increased queue size
        )

    def listener_callback(self, msg):
        start_time = time.time()
        # Send these velocities to the motors:
        try:
            self.esp.send_velocity_command(float(msg.linear.x), float(msg.angular.z))
            self.esp.read_serial_data(self.motorMessage)
        except Exception as e:
            self.get_logger().info(f"Error ${e}")

        end_time = time.time()

        self.get_logger().info(f"Linear Vel: {self.motorMessage.velocity_x}, Requested: {msg.linear.x}")
        self.get_logger().info(f"Angular Vel: {self.motorMessage.velocity_angular}, Requested: {msg.angular.z}")
        self.get_logger().info(f"Callback duration: {end_time - start_time:.4f} seconds")

    def destroy_node(self):
        super().destroy_node()  # Don't forget to call the base implementation

def main(args=None):
    rclpy.init(args=args)
    diff_drive_node = DiffDriveController()

    executor = MultiThreadedExecutor()
    executor.add_node(diff_drive_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        diff_drive_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()