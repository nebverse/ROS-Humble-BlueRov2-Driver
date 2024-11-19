#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink

from std_msgs.msg import UInt16, Bool
from sensor_msgs.msg import BatteryState
from bluerov2_interfaces.msg import Attitude, Bar30

class ControllerWithGripper(Node):
    
    def __init__(self):
        super().__init__("controller_with_gripper")              

        self.type               = mavlink.MAV_TYPE_GCS              
        self.autopilot          = mavlink.MAV_AUTOPILOT_INVALID     
        self.base_mode          = mavlink.MAV_MODE_PREFLIGHT        
        self.custom_mode        = 0                                 
        self.mavlink_version    = 0                                 
        self.heartbeat_period   = 0.02                              

        # RC Channel defaults
        self.pitch              = 1500                              
        self.roll               = 1500                              
        self.throttle           = 1500                              
        self.yaw                = 1500                              
        self.forward            = 1500                              
        self.lateral            = 1500                              
        self.camera_pan         = 1500                              
        self.camera_tilt        = 1500                              
        self.lights             = 1100                              
        self.gripper            = 1500  # Neutral value for gripper control

        self.data               = {}                                
       
        # Create subscribers
        self.rc_pitch_sub       = self.create_subscription(UInt16, "/bluerov2/rc/pitch", lambda msg: self.rc_callback(msg, 1), 10)
        self.rc_roll_sub        = self.create_subscription(UInt16, "/bluerov2/rc/roll", lambda msg: self.rc_callback(msg, 2), 10)
        self.rc_throttle_sub    = self.create_subscription(UInt16, "/bluerov2/rc/throttle", lambda msg: self.rc_callback(msg, 3), 10)
        self.rc_yaw_sub         = self.create_subscription(UInt16, "/bluerov2/rc/yaw", lambda msg: self.rc_callback(msg, 4), 10)
        self.rc_forward_sub     = self.create_subscription(UInt16, "/bluerov2/rc/forward", lambda msg: self.rc_callback(msg, 5), 10)
        self.rc_lateral_sub     = self.create_subscription(UInt16, "/bluerov2/rc/lateral", lambda msg: self.rc_callback(msg, 6), 10)
        self.rc_camera_pan_sub  = self.create_subscription(UInt16, "/bluerov2/rc/camera_pan", lambda msg: self.rc_callback(msg, 7), 10)
        self.rc_camera_tilt_sub = self.create_subscription(UInt16, "/bluerov2/rc/camera_tilt", lambda msg: self.rc_callback(msg, 8), 10)
        self.rc_lights_sub      = self.create_subscription(UInt16, "/bluerov2/rc/lights", lambda msg: self.rc_callback(msg, 9), 10)
        self.rc_gripper_sub     = self.create_subscription(UInt16, "/bluerov2/rc/gripper", lambda msg: self.rc_callback(msg, 10), 10)

        self.arm_sub            = self.create_subscription(Bool, "/bluerov2/arm", self.arm_callback, 10)
        
        # Create publishers
        self.battery_pub        = self.create_publisher(BatteryState, "/bluerov2/battery", 10)
        self.arm_pub            = self.create_publisher(Bool, "/bluerov2/arm_status", 10)  
        self.attitude_pub       = self.create_publisher(Attitude, "/bluerov2/attitude", 10)   
        self.bar30_pub          = self.create_publisher(Bar30, "/bluerov2/bar30", 10)             
        
        # Setup connection parameters
        self.declare_parameter("ip", "0.0.0.0") 
        self.declare_parameter("port", 14550)
        self.declare_parameter("baudrate", 115200)         

        self.bluerov_ip         = self.get_parameter("ip").value
        self.bluerov_port       = self.get_parameter("port").value  
        self.bluerov_baudrate   = self.get_parameter("baudrate").value

        self.get_logger().info("Controller with gripper for BlueROV2 started successfully!")

        # Create the connection to the BlueROV2
        self.connection = mavutil.mavlink_connection("udpin:" + self.bluerov_ip + ":" + str(self.bluerov_port), baudrate=self.bluerov_baudrate)        
        
        self.get_logger().info("Connecting to BlueROV2...")
        self.connection.wait_heartbeat()
        self.get_logger().info("BlueROV2 connection successful!")       

        # Convenience
        self.mav        = self.connection.mav
        self.recv_match = self.connection.recv_match
        self.target     = (self.connection.target_system,
                           self.connection.target_component)
        
        # Request data stream
        self.get_logger().info("Requesting data stream...") 
        self.mav.request_data_stream_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        
        # Arm the vehicle
        self.arm()

        # Start the update loop for BlueROV2 commands
        self.get_logger().info("Start sending heartbeat messages...")
        self.create_timer(self.heartbeat_period, self.send_bluerov_commands)

    def send_bluerov_commands(self):
        # Send regular heartbeats    
        self.connection.mav.heartbeat_send(self.type, 
                                           self.autopilot, 
                                           self.base_mode, 
                                           self.custom_mode, 
                                           self.mavlink_version)
        
        # Send RC channel updates
        rc_channel_values = (self.pitch,
                             self.roll,
                             self.throttle,
                             self.yaw,
                             self.forward,
                             self.lateral,
                             self.camera_pan,
                             self.camera_tilt,
                             self.lights,
                             self.gripper,  # Gripper RC channel
                             65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535) # Unused RC channels

        self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

    def rc_callback(self, msg, topic):
        match topic:
            case 1: self.pitch          = msg.data
            case 2: self.roll           = msg.data
            case 3: self.throttle       = msg.data
            case 4: self.yaw            = msg.data
            case 5: self.forward        = msg.data
            case 6: self.lateral        = msg.data
            case 7: self.camera_pan     = msg.data
            case 8: self.camera_tilt    = msg.data
            case 9: self.lights         = msg.data
            case 10: self.gripper       = msg.data  # Handle gripper RC input

    def arm_callback(self, msg):
        if msg.data:
            self.arm()
        else:
            self.disarm()

    def arm(self):
        self.connection.arducopter_arm()
        self.get_logger().info('Arm requested, waiting...')
        self.connection.motors_armed_wait()
        self.get_logger().info('Thrusters armed!')

    def disarm(self):
        self.connection.arducopter_disarm()
        self.get_logger().info('Disarm requested, waiting...')
        self.connection.motors_disarmed_wait()
        self.get_logger().info('Thrusters disarmed')    

def main(args=None):
    rclpy.init(args=args)    
    node = ControllerWithGripper()
    rclpy.spin(node)    

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
