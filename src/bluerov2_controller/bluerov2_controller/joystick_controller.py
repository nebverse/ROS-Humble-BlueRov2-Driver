import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        # Subscribe to joystick inputs
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel', 10)

        # Linear and angular scaling factors
        self.linear_scale = 0.5  # Scale for linear motion
        self.angular_scale = 0.2  # Scale for angular motion

    def joy_callback(self, joy_msg):
        cmd = Twist()

        # Map joystick axes to velocity
        cmd.linear.x = self.linear_scale * joy_msg.axes[1]  # Forward/Backward (e.g., Left Stick Y)
        cmd.linear.y = self.linear_scale * joy_msg.axes[0]  # Left/Right (e.g., Left Stick X)
        cmd.linear.z = self.linear_scale * joy_msg.axes[3]  # Up/Down (e.g., Right Stick Y)
        cmd.angular.z = self.angular_scale * joy_msg.axes[2]  # Yaw (e.g., Right Stick X)

        # Publish velocity commands
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f"Publishing: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
