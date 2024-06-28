import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class CustomTeleop(Node):

    def __init__(self):
        super().__init__('turtlebot_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.max_angular_speed = 10.0
        self.max_linear_speed = 1.0
        self.speed_increment_lin = 0.025
        self.speed_increment_ang = 0.15
        self.print_instructions()
        self.get_logger().info('Teleop Node Started. Use WASD keys to control the robot.')

    def print_instructions(self):
        print("""
Control TurtleBot!
---------------------------
Moving around:
   w    
a  s  d

i : switch to forward
k : switch to backward
j : switch to left turn
l : switch to right turn
x : full stop

q : quit
CTRL-C to quit
---------------------------
        """)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def run(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':
                    self.linear_speed = min(self.max_linear_speed, self.linear_speed + self.speed_increment_lin)
                elif key == 's':
                    self.linear_speed = max(-self.max_linear_speed, self.linear_speed - self.speed_increment_lin)
                elif key == 'a':
                    self.angular_speed = min(self.max_angular_speed, self.angular_speed + self.speed_increment_ang)
                elif key == 'd':
                    self.angular_speed = max(-self.max_angular_speed, self.angular_speed - self.speed_increment_ang)
                elif key == 'x':
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                elif key == 'i':
                    self.linear_speed = abs(self.linear_speed)
                elif key == 'k':
                    self.linear_speed = -abs(self.linear_speed)
                elif key == 'j':
                    self.angular_speed = abs(self.angular_speed)
                elif key == 'l':
                    self.angular_speed = -abs(self.angular_speed)
                elif key == 'q' or key == '\x03':  # CTRL-C to quit
                    break

                # Update twist message if new
                if (twist.linear.x != self.linear_speed or twist.angular.z != self.angular_speed): 
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed
                    self.publisher_.publish(twist)
                
                
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()