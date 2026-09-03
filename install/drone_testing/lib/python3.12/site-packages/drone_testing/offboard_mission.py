import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
import math

class MissionPlanner(Node):
    
    def __init__(self):
        super().__init__('mission_planner')
        
        # --- QoS Profile Definition ---
        # Sensor data profile matching PX4 MicroXRCEAgent defaults
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # --- Subscribers ---
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile=sensor_qos)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile=sensor_qos)
        
        # --- State Variables ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.local_position = VehicleLocalPosition()
        self.local_position_received = False
        self.offboard_setpoint_counter = 0

        # Mission target: 60 cm height -> Z = -0.6 m in local NED frame
        self.target_altitude = -0.6
        self.takeoff_start_x = 0.0
        self.takeoff_start_y = 0.0

        self.current_stage = "PREPARATION"
        self.hover_timer_counter = 0
        
        # --- Timer (10 Hz execution rate) ---
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.local_position = msg
        self.local_position_received = True
        
        # Lock initial ground position
        if self.current_stage == "PREPARATION" and msg.xy_valid and msg.z_valid and self.takeoff_start_x == 0.0:
            self.takeoff_start_x = msg.x
            self.takeoff_start_y = msg.y
            self.get_logger().info(f"Home location locked: X:{self.takeoff_start_x:.2f}, Y:{self.takeoff_start_y:.2f}")

    def timer_callback(self):
        # Always stream offboard control mode signal
        self.publish_offboard_control_mode()
        
        if self.current_stage == "PREPARATION":
            if not self.local_position_received:
                self.get_logger().info("Waiting for valid local position...", throttle_duration_sec=2.0)
                self.publish_trajectory_setpoint(x=0.0, y=0.0, z=0.0)
                return
                
            # Stream current ground position setpoints to warm up PX4 offboard stream
            self.publish_trajectory_setpoint(x=self.local_position.x, y=self.local_position.y, z=self.local_position.z)
            if self.offboard_setpoint_counter < 10:
                self.offboard_setpoint_counter += 1
            else:
                self.current_stage = "ARMING"
        
        elif self.current_stage == "ARMING":
            self.get_logger().info("Stage: ARMING", throttle_duration_sec=2.0)
            if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Vehicle is armed.")
                self.current_stage = "TAKEOFF"
            else:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                
        elif self.current_stage == "TAKEOFF":
            self.get_logger().info("Stage: TAKEOFF to 60 cm", throttle_duration_sec=2.0)
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # Command setpoint at X_home, Y_home, Altitude = 60 cm (Z = -0.6 m)
                self.publish_trajectory_setpoint(x=self.takeoff_start_x, y=self.takeoff_start_y, z=self.target_altitude)
                
                # Check if drone reached altitude within 10 cm proximity
                if abs(self.local_position.z - self.target_altitude) < 0.10:
                    self.get_logger().info("Reached 60 cm height. Holding position for 6 seconds...")
                    self.current_stage = "HOVER_6_SEC"
            else:
                self.get_logger().info("Requesting Offboard mode...", throttle_duration_sec=2.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

        elif self.current_stage == "HOVER_6_SEC":
            # Continue streaming position setpoint at 60 cm height
            self.publish_trajectory_setpoint(x=self.takeoff_start_x, y=self.takeoff_start_y, z=self.target_altitude)
            
            # Timer runs at 10 Hz -> 60 iterations = 6.0 seconds
            self.hover_timer_counter += 1
            if self.hover_timer_counter >= 60:
                self.get_logger().info("6 seconds hover completed. Initiating Landing.")
                self.current_stage = "LANDING"

        elif self.current_stage == "LANDING":
            self.get_logger().info("Stage: LANDING", throttle_duration_sec=2.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.current_stage = "DISARMED_LANDED"
            
        elif self.current_stage == "DISARMED_LANDED":
            self.get_logger().info("Stage: LANDED & DISARMED. Mission Complete.", throttle_duration_sec=5.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position[0] = float(x)
        msg.position[1] = float(y)
        msg.position[2] = float(z)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mission_planner = MissionPlanner()
    try:
        rclpy.spin(mission_planner)
    except KeyboardInterrupt:
        pass
    finally:
        mission_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()