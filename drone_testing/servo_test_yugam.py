#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Bool, Float32
import math

class ServoSoftwareOverrideNode(Node):
    def __init__(self):
        super().__init__('servo_software_override_node')
        
        # Publisher to PX4 via DDS
        self.command_pub = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            10
        )
        
        # Subscribers for Jetson-side Terminal Override
        self.override_sub = self.create_subscription(Bool, '/override_toggle', self.toggle_cb, 10)
        self.val_sub = self.create_subscription(Float32, '/override_value', self.val_cb, 10)
        
        # State Variables
        self.override_active = False
        self.manual_servo_val = -1.0
        self.jetson_target_value = -1.0 
        
        # Timer loop (10Hz) to continuously send the actuator command
        self.timer = self.create_timer(0.1, self.publish_servo_command)
        self.get_logger().info("Software Override Node Started. Defaulting to AUTO mode...")
        self.get_logger().info("Use 'ros2 topic pub' to /override_toggle to take manual control.")

    def toggle_cb(self, msg: Bool):
        """Activates or deactivates the manual override."""
        self.override_active = msg.data
        state = "MANUAL OVERRIDE" if self.override_active else "AUTO MODE"
        self.get_logger().info(f"Mode switched to: {state}")

    def val_cb(self, msg: Float32):
        """Receives the manual servo position and clamps it to valid ranges."""
        self.manual_servo_val = max(-1.0, min(1.0, msg.data))
        if self.override_active:
            self.get_logger().info(f"Manual Servo Value set to: {self.manual_servo_val}")

    def publish_servo_command(self):
        """Evaluates state and pushes the command to the Pixhawk."""
        
        # 1. AUTONOMOUS LOGIC
        # (Insert your vision/sensor processing here. Updating self.jetson_target_value)
        self.jetson_target_value = -1.0 

        # 2. DETERMINE ACTIVE VALUE
        if self.override_active:
            active_val = self.manual_servo_val
        else:
            active_val = self.jetson_target_value
            
        # 3. PUBLISH TO PX4
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        
        # Target "Peripheral via actuator set 1 (Output 5)"
        cmd.param1 = math.nan
        cmd.param2 = math.nan
        cmd.param3 = math.nan
        cmd.param4 = math.nan
        cmd.param5 = float(active_val) # Output 5
        cmd.param6 = math.nan
        cmd.param7 = 0.0 
        
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True

        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ServoSoftwareOverrideNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()