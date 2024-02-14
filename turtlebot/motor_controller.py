import rclpy
from turtlebot.espComms import ESPcomms 
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        #esp comms init 
        self.esp = ESPcomms()
        self.esp.scan_devices()
        self.esp.port = self.esp.scan_devices()[0] #for selecting correct usb port for esp32 (should be top right)
        connectBool = self.esp.initSTMhandshake()
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

        # Parameters for differential drive calculations
        self.wheel_radius = 0.067  # meters
        self.wheel_separation = 0.18  # meters

    def listener_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Convert Twist velocities to left and right wheel velocities
        left_wheel_velocity, right_wheel_velocity = self.twist_to_wheel_velocities(linear_velocity, angular_velocity)

        #print motor logs
        self.get_logger().info(f"Left Wheel Velocity: {left_wheel_velocity}, Right Wheel Velocity: {right_wheel_velocity}")

        #Send these velocities to the motors:
        sendBool = self.esp.send_data("%s,%s\n"%(left_wheel_velocity,right_wheel_velocity))
        if not sendBool:
            self.get_logger().info("Error sending data!")

    def twist_to_wheel_velocities(self, linear_velocity, angular_velocity):
        # Calculate wheel velocities based on differential drive kinematics
        v_left = linear_velocity - (self.wheel_separation / 2.0) * angular_velocity
        v_right = linear_velocity + (self.wheel_separation / 2.0) * angular_velocity
        return v_left, v_right
    
    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping motors...")
        self.esp.send_data("0,0\n")  # Send command to stop motors
        super().destroy_node()  # Don't forget to call the base implementation


def main(args=None):

    rclpy.init(args=args)
    diff_drive_node = DiffDriveController()

    rclpy.spin(diff_drive_node)

    diff_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
