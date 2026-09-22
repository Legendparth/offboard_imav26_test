import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors, OffboardControlMode

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Publishers
        self.servo_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.position_is_zero = True
        self.counter = 0

    def timer_callback(self):
        # 1. Maintain Offboard Mode Heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.direct_actuator = True
        self.offboard_mode_pub.publish(offboard_msg)

        # 2. Toggle position every 2 seconds
        self.counter += 1
        if self.counter >= 20:
            self.position_is_zero = not self.position_is_zero
            self.counter = 0

        # 3. Publish to ActuatorMotors (maps to Actuator Set)
        motor_msg = ActuatorMotors()
        motor_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        motor_msg.control = [float('nan')] * 12
        
        if self.position_is_zero:
            motor_msg.control[0] = -1.0  # Min position
        else:
            motor_msg.control[0] = 1.0   # Max position
            
        self.servo_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()