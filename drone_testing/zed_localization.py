import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import time

class ZedLocalization(Node):
    def __init__(self):
        super().__init__('zed_localization')
        self.get_logger().info("ZED Localization Node starting")

        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -- subscriber --
        self.zed_odom_sub = self.create_subscription(Odometry, '/zed/zed_node/odom', self.zed_odom_callback, qos_profile_sensor_data)
        # -- publisher --
        self.vehicle_visual_odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos_profile)
    

    def zed_odom_callback(self, zed_msg):
        px4_msg = VehicleOdometry()
        
        # Sync timestamp in microseconds
        px4_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        px4_msg.timestamp_sample = px4_msg.timestamp
        
        # Frame constants (NED)
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        
        # Position Translation: ENU to NED
        px4_msg.position = [
            zed_msg.pose.pose.position.y,
            zed_msg.pose.pose.position.x,
            -zed_msg.pose.pose.position.z
        ]
        
        # Quaternion Translation: ENU to NED
        px4_msg.q = [
            zed_msg.pose.pose.orientation.w,
            zed_msg.pose.pose.orientation.y,
            zed_msg.pose.pose.orientation.x,
            -zed_msg.pose.pose.orientation.z
        ]
        
        # Velocity Translation: ENU to NED
        px4_msg.velocity = [
            zed_msg.twist.twist.linear.y,
            zed_msg.twist.twist.linear.x,
            -zed_msg.twist.twist.linear.z
        ]
        
        # Angular Velocity Translation: ENU to NED
        px4_msg.angular_velocity = [
            zed_msg.twist.twist.angular.y,
            zed_msg.twist.twist.angular.x,
            -zed_msg.twist.twist.angular.z
        ]
        
        self.vehicle_visual_odom_pub.publish(px4_msg)

    


def main(args=None):
    rclpy.init(args=args)
    zed_localization_node = ZedLocalization()
    try:
        rclpy.spin(zed_localization_node)
    except KeyboardInterrupt:
        zed_localization_node.get_logger().info("Shutdown requested.")
    finally:
        zed_localization_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
