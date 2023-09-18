#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node

from bluerov2_interfaces.msg import Attitude
from bluerov2_interfaces.msg import SetRoll
from bluerov2_interfaces.msg import SetTarget

from std_msgs.msg import UInt16

class Controller(Node):

    def __init__(self):
        super().__init__("roll controller")

        # Setup default parameters
        self.declare_parameter("roll_desired", 0) 
        self.declare_parameter("pwm_max", 1900)            
        self.declare_parameter("kp", 35)    
        self.declare_parameter("kd", 25)    
        self.declare_parameter("enable", True)  

        self.attitude           = [0, 0, 0, 0, 0, 0]                            #[ROLL, PITCH, YAW, ROLLSPEED, PITCHSPEED, YAWSPEED]
        self.pwm_max            = self.get_parameter("pwm_max").value           # Maximum PWM value
        self.pwm_neutral        = 1500                                          # Neutral PWM value
        self.roll_desired       = self.get_parameter("roll_desired").value      # Desired pitch setpoint
        self.KP                 = self.get_parameter("kp").value                # Proportional gain constant        
        self.KD                 = self.get_parameter("kd").value                # Derivative gain constant

        self.enable             = self.get_parameter("enable").value   

        # Create subscriber
        self.attitude_sub       = self.create_subscription(Attitude, "/bluerov2/attitude", self.callback_att, 10) 
        self.setRoll_sub        = self.create_subscription(SetRoll, "/settings/set_roll", self.callback_set_roll, 10)
        self.setTarget_sub      = self.create_subscription(SetTarget, "/settings/set_target", self.callback_set_target, 10) 

        # Create publisher
        self.roll_pub           = self.create_publisher(UInt16, "/bluerov2/rc/roll", 10)

        # Start update loop
        self.create_timer(0.04, self.calculate_pwm)    

    def callback_att(self, msg):       
        self.attitude = [msg.roll,
                         msg.pitch,
                         msg.yaw,
                         msg.rollspeed,
                         msg.pitchspeed,
                         msg.yawspeed]

    def callback_set_roll(self, msg):       
        if msg.pwm_max < 1500:
            self.pwm_max = 1500
        else:
            self.pwm_max = msg.pwm_max
        self.KP = msg.kp 
        self.KD = msg.kd

        self.enable = msg.enable_roll_ctrl

    def callback_set_target(self, msg):       
        self.roll_desired = self.deg2rad(msg.roll_desired)

    def deg2rad(self,deg):       
        if deg in range(0,181):
            return (deg * np.pi) / 180 
        if deg in range(181,361):
            return ((deg - 360) * np.pi) / 180

    def control(self, roll, rollspeed):        
        return self.KP*self.sawtooth(roll-self.roll_desired) + self.KD*rollspeed
    
    def saturation(self, pwm):        
        pwm_min = self.pwm_neutral - (self.pwm_max - self.pwm_neutral)
        if pwm > self.pwm_max :
            pwm = self.pwm_max
        if pwm < pwm_min:
            pwm = pwm_min
        return int(pwm)
    
    def sawtooth (self, x):
        """Deal with 2*PI modulo
        
        Input:
        ------
        x: rad 
        """
        return (x+np.pi)%(2*np.pi)-np.pi      
    
    def calculate_pwm(self): 
        msg = UInt16()

        if self.enable:
            roll = self.attitude[0]
            rollspeed = self.attitude[3]
            u = self.control(roll, rollspeed)
            pwm = self.pwm_neutral - u
            pwm = self.saturation(pwm)
            
            msg.data = pwm
        else:
            msg.data = self.pwm_neutral
            
        self.roll_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)    
    node = Controller()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()