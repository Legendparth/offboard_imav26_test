import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorServos

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        # Publisher for the actuator set commands going into PX4
        self.publisher = self.create_publisher(
            ActuatorServos,
            '/fmu/in/actuator_servos',
            10
        )
        # Timer to toggle position every 2 seconds for testing
        self.timer = self.create_timer(2.0, self.toggle_servo)
        self.position_is_zero = True

    def toggle_servo(self):
        msg = ActuatorServos()
        # PX4 requires a synced timestamp in microseconds
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Initialize array with NaNs to ignore other servos
        msg.control = [float('nan')] * 8
        
        # control[0] maps to "Peripheral via actuator set 1"
        if self.position_is_zero:
            msg.control[0] = -1.0  # Min PWM (0 degrees)
            self.get_logger().info('Moving servo to 0 degrees')
        else:
            msg.control[0] = 0.0   # Center PWM (90 degrees)
            self.get_logger().info('Moving servo to 90 degrees')
            
        self.position_is_zero = not self.position_is_zero
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()