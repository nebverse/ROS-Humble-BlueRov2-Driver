#!/usr/bin/env python3
import rclpy
import pygame
from pygame.locals import *
from rclpy.node import Node
from std_msgs.msg import UInt16, Bool

class BasicInputController(Node):
    def __init__(self):
        super().__init__("input_controller_basic")

        # Initialize parameters
        self.declare_parameter("pwm_max", 1900)
        self.declare_parameter("pwm_min", 1100)
        self.declare_parameter("pwm_neutral", 1500)

        self.pwm_max = self.get_parameter("pwm_max").value
        self.pwm_min = self.get_parameter("pwm_min").value
        self.pwm_neutral = self.get_parameter("pwm_neutral").value

        # State variables for ROV controls
        self.arm_status = False

        # Publishers for all control topics
        self.pitch_pub = self.create_publisher(UInt16, "/bluerov2/rc/pitch", 10)
        self.roll_pub = self.create_publisher(UInt16, "/bluerov2/rc/roll", 10)
        self.throttle_pub = self.create_publisher(UInt16, "/bluerov2/rc/throttle", 10)
        self.yaw_pub = self.create_publisher(UInt16, "/bluerov2/rc/yaw", 10)
        self.forward_pub = self.create_publisher(UInt16, "/bluerov2/rc/forward", 10)
        self.lateral_pub = self.create_publisher(UInt16, "/bluerov2/rc/lateral", 10)
        self.camera_pan_pub = self.create_publisher(UInt16, "/bluerov2/rc/camera_pan", 10)
        self.camera_tilt_pub = self.create_publisher(UInt16, "/bluerov2/rc/camera_tilt", 10)
        self.lights_pub = self.create_publisher(UInt16, "/bluerov2/rc/lights", 10)
        self.gripper_pub = self.create_publisher(UInt16, "/bluerov2/rc/gripper", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm", 10)

        # Initialize joystick
        pygame.init()
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Joystick detected: {self.joystick.get_name()}")
        else:
            self.get_logger().error("No joystick detected!")
            return

        # Start update loop
        self.create_timer(0.04, self.update_input)

    def update_input(self):
        if self.joystick is None:
            return

        pygame.event.pump()

        # Read joystick axes
        self.publish_axis(self.joystick.get_axis(0), self.lateral_pub, "Lateral")
        self.publish_axis(self.joystick.get_axis(1), self.forward_pub, "Forward")
        self.publish_axis(self.joystick.get_axis(2), self.yaw_pub, "Yaw")
        self.publish_axis(self.joystick.get_axis(3), self.throttle_pub, "Throttle")

        # Read D-pad (hat) for camera control
        hat = self.joystick.get_hat(0)
        self.publish_dpad(hat)

        # Read buttons
        for button in range(self.joystick.get_numbuttons()):
            if self.joystick.get_button(button):
                self.handle_button_press(button)

    def publish_axis(self, value, publisher, name):
        pwm = self.calculate_pwm(value)
        msg = UInt16(data=pwm)
        publisher.publish(msg)
        self.get_logger().debug(f"Published {name}: PWM={pwm}")

    def publish_dpad(self, hat):
        # Camera pan (D-pad left/right) and tilt (D-pad up/down)
        pan_pwm = self.calculate_pwm(hat[0])  # Horizontal (left/right)
        tilt_pwm = self.calculate_pwm(hat[1])  # Vertical (up/down)

        pan_msg = UInt16(data=pan_pwm)
        tilt_msg = UInt16(data=tilt_pwm)

        self.camera_pan_pub.publish(pan_msg)
        self.camera_tilt_pub.publish(tilt_msg)

        self.get_logger().debug(f"Published Camera Pan: PWM={pan_pwm}")
        self.get_logger().debug(f"Published Camera Tilt: PWM={tilt_pwm}")

    def handle_button_press(self, button):
        if button == 4:  # Left Bumper (LB) for lights decrease
            self.adjust_lights(-50)
        elif button == 5:  # Right Bumper (RB) for lights increase
            self.adjust_lights(50)
        elif button == 0:  # A Button to toggle gripper open/close
            self.toggle_gripper()
        elif button == 7:  # Start Button to toggle arm/disarm
            self.toggle_arm()

    def adjust_lights(self, delta):
        current_lights = self.pwm_neutral + delta
        lights_pwm = max(self.pwm_min, min(self.pwm_max, current_lights))
        msg = UInt16(data=lights_pwm)
        self.lights_pub.publish(msg)
        self.get_logger().info(f"Adjusted Lights: PWM={lights_pwm}")

    def toggle_gripper(self):
        gripper_pwm = self.pwm_min if self.arm_status else self.pwm_max
        msg = UInt16(data=gripper_pwm)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Toggled Gripper: PWM={gripper_pwm}")

    def toggle_arm(self):
        self.arm_status = not self.arm_status
        msg = Bool(data=self.arm_status)
        self.arm_pub.publish(msg)
        status = "armed" if self.arm_status else "disarmed"
        self.get_logger().info(f"Thrusters {status}")

    def calculate_pwm(self, value):
        return int(self.pwm_neutral + value * (self.pwm_max - self.pwm_neutral))

def main(args=None):
    rclpy.init(args=args)
    node = BasicInputController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Input Controller Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
