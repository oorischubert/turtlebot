import rclpy
from turtlebot.motorComms import MotorController, MotorMessage
import configuration as config
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        #esp comms init 
        self.esp = MotorController(config.ESP_SERIAL_PORT)
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
            10
        )

    def listener_callback(self, msg):
    
        #Send these velocities to the motors:
        self.esp.send_velocity_command(msg.linear.x, msg.angular.z)
        self.esp.read_serial_data(self.motorMessage)

        self.get_logger().info(f"Linear Vel: {self.motorMessage.velocity_x}, Requested: {msg.linear.x}")
        self.get_logger().info(f"Angular Vel: {self.motorMessage.velocity_angular}, Requested: {msg.angular.z}")
        

    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping motors...")
        self.esp.shutdown()
        super().destroy_node()  # Don't forget to call the base implementation


def main(args=None):

    rclpy.init(args=args)
    diff_drive_node = DiffDriveController()

    rclpy.spin(diff_drive_node)

    diff_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
