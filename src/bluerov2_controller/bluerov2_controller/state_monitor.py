import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State

class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)

    def state_callback(self, msg):
        self.get_logger().info(f"Connected: {msg.connected}, Armed: {msg.armed}, Mode: {msg.mode}")

def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# 2. Configure MAVLink on the Companion Computer

# Access the BlueROV2 companion computer (Raspberry Pi) via SSH:

# ssh pi@192.168.2.2

# Edit MAVLink Endpoint Configuration

#     Open the MAVLink configuration file:

# nano /home/pi/companion/.companion.rc

# Add a new endpoint for your ROS2 machine:

# MAVLINK_ROUTERD_ENDPOINT_1=192.168.2.1:14550   # QGC
# MAVLINK_ROUTERD_ENDPOINT_2=192.168.2.100:14555 # ROS2 node (replace with your ROS2 computer's IP)

# Restart the MAVLink service:

#     sudo systemctl restart mavlink-router

# Now, the MAVLink messages will be broadcast to both QGC and the specified ROS2 endpoint.
# 3. Launch MAVROS in ROS2

# On your ROS2 machine, run the mavros node and point it to the secondary MAVLink endpoint:

# ros2 launch mavros mavros.launch.py fcu_url:="udp://192.168.2.2:14555@192.168.2.100:14555"

# Replace 192.168.2.100 with your ROS2 machine’s IP.
# 4. Verify Connections

#     QGroundControl:
#         Ensure QGroundControl remains connected and telemetry data is visible.

#     ROS2:
#         Confirm the mavros node is running correctly by checking topics:

#         ros2 topic list

#         Key MAVROS topics:
#             /mavros/state
#             /mavros/setpoint_velocity/cmd_vel
#             /mavros/set_mode
#             /mavros/cmd/arming

# 5. Run Parallel ROS2 Scripts

# You can now run additional ROS2 scripts in parallel with QGC for other tasks. For example:

# 6. Important Notes

#     Mode Switching:
#         Ensure the BlueROV2 is in GUIDED mode for automated tasks or MANUAL mode for teleoperation.
#         Use QGC for mode management, or switch modes using MAVROS:

#         ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"

#     Telemetry Bandwidth:
#         Ensure sufficient network bandwidth, as routing MAVLink messages to multiple endpoints can increase data usage.

#     Prioritize Manual Control:
#         If a script conflicts with manual control, QGC commands should take precedence. Avoid scripts that publish conflicting commands (e.g., to /mavros/setpoint_velocity/cmd_vel).

# 7. Testing

#     Start QGC and verify telemetry and manual control.
#     Launch the mavros node on your ROS2 machine.
#     Run your ROS2 scripts to perform parallel tasks (e.g., monitoring, logging, or auxiliary operations).
#     Verify both QGC and ROS2 can interact without disconnecting or conflicting.

