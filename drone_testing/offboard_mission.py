import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
import time
import math

class MissionPlanner(Node):
    
    def __init__(self):
        super().__init__('mission_planner')
        
        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # --- Subscribers ---
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile=qos_profile_sensor_data)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile=qos_profile_sensor_data)
        
        # --- State Variables ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.local_position = VehicleLocalPosition()
        self.local_position_received = False
        self.offboard_setpoint_counter = 0

        self.mission_sequence = []
        self.current_waypoint_index = 0
        
        self.takeoff_altitude = -0.1
        self.takeoff_start_x = 0.0
        self.takeoff_start_y = 0.0

        self.current_stage = "PREPARATION"
        self.takeoff_reached_counter = 0
        self.waypoint_reached_counter = 0
        
        # --- Timers ---
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.local_position = msg
        self.local_position_received = True
        # Lock in the home coordinate only once during preparation
        if self.current_stage == "PREPARATION" and msg.xy_valid and msg.z_valid and self.takeoff_start_x == 0.0:
            self.takeoff_start_x = msg.x
            self.takeoff_start_y = msg.y
            
            # Now that we know where home is, build the exact coordinates for the mission
            self.mission_sequence = [
                {"name": "Forward 1m",    "x": self.takeoff_start_x + 1.0, "y": self.takeoff_start_y,       "z": -0.5},
                {"name": "Return",        "x": self.takeoff_start_x,       "y": self.takeoff_start_y,       "z": -0.5},
                {"name": "Up to 1.0m",    "x": self.takeoff_start_x,       "y": self.takeoff_start_y,       "z": -1.0},
                {"name": "Down to 0.5m",  "x": self.takeoff_start_x,       "y": self.takeoff_start_y,       "z": -0.5},
                {"name": "Right 1m",      "x": self.takeoff_start_x,       "y": self.takeoff_start_y + 1.0, "z": -0.5},
                {"name": "Return",        "x": self.takeoff_start_x,       "y": self.takeoff_start_y,       "z": -0.5},
            ]
            self.get_logger().info(f"Home position locked at X:{self.takeoff_start_x:.2f}, Y:{self.takeoff_start_y:.2f}")

    def timer_callback(self):
        # Always publish offboard control mode
        self.publish_offboard_control_mode()
        
        if self.current_stage == "PREPARATION":
            if not self.local_position_received:
                self.get_logger().info("Waiting for valid local position...", throttle_duration_sec=2.0)
                self.publish_trajectory_setpoint(x=0.0, y=0.0, z=0.0)
                return
                
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
            self.get_logger().info("Stage: TAKEOFF", throttle_duration_sec=2.0)
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_trajectory_setpoint(x=self.takeoff_start_x, y=self.takeoff_start_y, z=self.takeoff_altitude)
                
                # Check altitude proximity
                if abs(self.local_position.z - self.takeoff_altitude) < 0.1:
                    self.takeoff_reached_counter += 1
                    if self.takeoff_reached_counter > 20:
                        self.get_logger().info("Takeoff altitude stabilized. Starting mission.")
                        self.current_stage = "EXECUTE_MISSION"
                else:
                    self.takeoff_reached_counter = 0
            else:
                self.get_logger().info("Switching to Offboard mode...", throttle_duration_sec=2.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

        elif self.current_stage == "EXECUTE_MISSION":
            if self.current_waypoint_index >= len(self.mission_sequence):
                self.get_logger().info("Mission sequence complete. Initiating Landing.")
                self.current_stage = "LANDING"
                return

            target = self.mission_sequence[self.current_waypoint_index]
            self.publish_trajectory_setpoint(x=target["x"], y=target["y"], z=target["z"])
            
            # Calculate 3D distance to target
            distance_to_target = math.sqrt(
                (self.local_position.x - target["x"])**2 +
                (self.local_position.y - target["y"])**2 +
                (self.local_position.z - target["z"])**2
            )
                                  
            # Tighter proximity check for 1-meter movements
            if distance_to_target < 0.15:
                self.waypoint_reached_counter += 1
                if self.waypoint_reached_counter > 20:
                    self.get_logger().info(f"Reached: {target['name']}")
                    self.current_waypoint_index += 1
                    self.waypoint_reached_counter = 0
            else:
                self.waypoint_reached_counter = 0
                
        elif self.current_stage == "LANDING":
            self.get_logger().info("Stage: LANDING", throttle_duration_sec=2.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.current_stage = "LANDED"
            
        elif self.current_stage == "LANDED":
            self.get_logger().info("Stage: LANDED. Mission Complete.", throttle_duration_sec=5.0)

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
