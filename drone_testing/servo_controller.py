import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
import math

class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')
        # Publisher for sending commands to PX4
        self.publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # Timer to publish commands at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        # Generate a smooth sine wave value between -1.0 and 1.0
        elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        servo_position = math.sin(elapsed_time * 2.0) 

        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        
        # param1 targets Actuator Set 1 (which is mapped to MAIN 5 in your QGC)
        msg.param1 = servo_position 
        
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.publisher.publish(msg)
        self.get_logger().info(f'Commanding MAIN 5 Servo to: {servo_position:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()