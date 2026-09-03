#!/usr/bin/env python3
"""
Offboard takeoff / hold / land test.

Sequence: stream offboard setpoints -> enter Offboard -> arm -> sit armed
on the ground for 5 s -> climb to 0.80 m above the arming point -> hold
there for 15 s -> descend slowly -> disarm once landed.

Altitude is flown as a position setpoint relative to wherever the vehicle
was standing when it armed. Horizontal is flown as a ZERO VELOCITY
setpoint, not a position setpoint: on an optical-flow airframe the x/y
position estimate on the ground is dead-reckoned garbage, and holding a
position latched down there makes the vehicle fly out the accumulated
error the moment flow starts correcting it. "Stay still" has no memory
and cannot do that.

Once the vehicle is at altitude with healthy flow, x/y position hold is
latched onto a FRESH estimate for the duration of the hold, which removes
the slow creep that a pure velocity hold has. The descent drops back to
velocity hold, because flow degrades again near the ground.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).

The vertical estimate must be healthy for this to be safe. The node
refuses to arm without z_valid.
"""

import select
import sys
import threading
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardTakeoff(Node):

    PREPARATION = "PREPARATION"
    OFFBOARD_REQUEST = "OFFBOARD_REQUEST"
    ARMING = "ARMING"
    GROUND_WAIT = "GROUND_WAIT"
    TAKEOFF = "TAKEOFF"
    HOLD = "HOLD"
    LANDING = "LANDING"
    DISARMING = "DISARMING"
    KILLING = "KILLING"
    DONE = "DONE"

    # ---- flight parameters -----------------------------------------------
    TAKEOFF_ALTITUDE = 0.80     # m above the arming point
    GROUND_WAIT_SECONDS = 5.0   # armed on the ground before the climb starts
    HOLD_SECONDS = 15.0         # station keeping once the altitude is reached
    CLIMB_SPEED = 0.35          # m/s, rate the climb setpoint is ramped at
    LAND_SPEED = 0.15           # m/s, rate the descent setpoint is ramped at
    ALTITUDE_TOLERANCE = 0.08   # m, "we are there" band around the target
    SETTLE_SECONDS = 0.5        # time inside the band before the hold starts
    LAND_OVERSHOOT = 0.50       # m the descent setpoint is pushed below ground

    # ---- flow / estimator health -----------------------------------------
    # Below this AGL the rangefinder and optical flow are not trustworthy:
    # too close for the lidar, too little parallax for the flow.
    FLOW_MIN_AGL = 0.30
    # x/y position hold is only latched after flow has been continuously
    # healthy this long, so a single good sample cannot trigger it.
    FLOW_SETTLE_SECONDS = 1.0

    # ---- timings / limits -------------------------------------------------
    SETPOINT_WARMUP = 20        # setpoints streamed before requesting Offboard (@20 Hz = 1 s)
    OFFBOARD_TIMEOUT = 10.0
    ARMING_TIMEOUT = 10.0
    TAKEOFF_TIMEOUT = 20.0
    LANDING_TIMEOUT = 30.0
    DISARM_TIMEOUT = 5.0
    LANDED_CONFIRM_SECONDS = 1.0    # land-detector must agree this long
    # If you switch to Offboard from your RC transmitter instead of from
    # this node, set this to False.
    REQUEST_OFFBOARD_FROM_ROS = True
    # ----------------------------------------------------------------------

    def __init__(self):
        super().__init__('offboard_takeoff')

        # The numbers you actually want to change between hardware tests are
        # exposed as ROS parameters; the rest stay as class constants above.
        # e.g. ros2 run ... --ros-args -p takeoff_altitude:=0.30
        self.TAKEOFF_ALTITUDE = self.declare_parameter(
            'takeoff_altitude', self.TAKEOFF_ALTITUDE).value
        self.HOLD_SECONDS = self.declare_parameter(
            'hold_seconds', self.HOLD_SECONDS).value
        self.GROUND_WAIT_SECONDS = self.declare_parameter(
            'ground_wait_seconds', self.GROUND_WAIT_SECONDS).value
        self.CLIMB_SPEED = self.declare_parameter(
            'climb_speed', self.CLIMB_SPEED).value
        self.LAND_SPEED = self.declare_parameter(
            'land_speed', self.LAND_SPEED).value
        self.REQUEST_OFFBOARD_FROM_ROS = self.declare_parameter(
            'request_offboard_from_ros', self.REQUEST_OFFBOARD_FROM_ROS).value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile=sensor_qos)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback, qos_profile=sensor_qos)

        # The land detector topic is unversioned on some builds and _v1 on
        # others; subscribe to both and take whichever one actually arrives.
        self.land_detected_subs = [
            self.create_subscription(
                VehicleLandDetected, topic, self.land_detected_callback,
                qos_profile=sensor_qos)
            for topic in ('/fmu/out/vehicle_land_detected',
                          '/fmu/out/vehicle_land_detected_v1')
        ]

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.status_received = False
        self._last_nav_state = None

        self.local_position = None
        self.landed = True
        self.land_detector_seen = False

        # Captured at the moment of arming; every setpoint is relative to it.
        self.home_x = None
        self.home_y = None
        self.home_z = None
        self.home_yaw = 0.0

        self.target_z = None        # NED z the ramp is currently walking towards
        self.setpoint_z = None      # NED z actually being commanded right now
        self.in_band_since = None
        self.landed_since = None

        # Horizontal control. hold_xy False -> command zero velocity;
        # True -> hold hold_x/hold_y, latched once airborne with good flow.
        self.hold_xy = False
        self.hold_x = None
        self.hold_y = None
        self.flow_healthy_since = None

        self.setpoint_counter = 0
        self.stage_enter_time = time.monotonic()
        self.current_stage = self.PREPARATION
        self.abort_requested = False
        self.kill_requested = False

        self._stop_event = threading.Event()
        self._stdin_is_tty = False
        self._stdin_old_settings = None
        self._keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._keyboard_thread.start()

        # 20 Hz. PX4 drops Offboard if setpoints arrive slower than 2 Hz.
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().warning(
            f"Takeoff test: {self.TAKEOFF_ALTITUDE:.2f} m, "
            f"{self.HOLD_SECONDS:.0f} s hold. Press q to abort into a descent, "
            "k to force-disarm.")

    # ------------------------------------------------------------------ subs

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.status_received = True

        if self.nav_state != self._last_nav_state:
            self._last_nav_state = self.nav_state
            self.get_logger().info(f"nav_state -> {self.nav_state}")

    def local_position_callback(self, msg):
        self.local_position = msg

    def land_detected_callback(self, msg):
        self.landed = msg.landed
        self.land_detector_seen = True

    def position_is_usable(self):
        lp = self.local_position
        return lp is not None and lp.xy_valid and lp.z_valid

    def flow_is_healthy(self):
        """Is the optical flow actually correcting, or just dead-reckoning?

        xy_valid alone is not enough -- EKF2 keeps it true while coasting on
        the IMU. Require a live velocity estimate and a rangefinder reading
        far enough off the ground for the flow to see anything.
        """
        lp = self.local_position
        return (lp is not None and lp.xy_valid and lp.v_xy_valid
                and lp.dist_bottom_valid and lp.dist_bottom > self.FLOW_MIN_AGL)

    def flow_healthy_for(self):
        """Seconds the flow has been continuously healthy, 0.0 if it is not."""
        if not self.flow_is_healthy():
            self.flow_healthy_since = None
            return 0.0
        if self.flow_healthy_since is None:
            self.flow_healthy_since = time.monotonic()
        return time.monotonic() - self.flow_healthy_since

    def relative_altitude(self):
        """Height above the arming point, positive up. None if unknown."""
        if self.home_z is None or self.local_position is None:
            return None
        return self.home_z - self.local_position.z

    def log_flight_state(self):
        lp = self.local_position
        if lp is None:
            self.get_logger().warning("No VehicleLocalPosition being published at all.",
                                      throttle_duration_sec=2.0)
            return

        alt = self.relative_altitude()
        alt_str = f"{alt:+.2f} m" if alt is not None else "n/a"
        self.get_logger().info(
            f"alt={alt_str} xy_valid={lp.xy_valid} z_valid={lp.z_valid} | "
            f"dist_bottom={lp.dist_bottom:.2f} m valid={lp.dist_bottom_valid} | "
            f"vz={lp.vz:+.2f} m/s landed={self.landed} | "
            f"xy={'POS-HOLD' if self.hold_xy else 'VEL-HOLD'} "
            f"flow_ok={self.flow_is_healthy()} "
            f"vxy=({lp.vx:+.2f},{lp.vy:+.2f})",
            throttle_duration_sec=1.0)

    # -------------------------------------------------------------- keyboard

    def _keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().warning("stdin is not a tty; q/k abort is unavailable.")
            return

        fd = sys.stdin.fileno()
        try:
            self._stdin_is_tty = True
            self._stdin_old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)

            while not self._stop_event.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                ch = sys.stdin.read(1).lower()
                if ch == 'q':
                    self.abort_requested = True
                    self.get_logger().warning("Abort requested: descending now.")
                elif ch == 'k':
                    self.kill_requested = True
                    self.get_logger().error("FORCE DISARM requested from keyboard.")
                    return
        except Exception as exc:
            self.get_logger().warning(f"Keyboard listener disabled: {exc}")
        finally:
            self._restore_stdin()

    def _restore_stdin(self):
        if self._stdin_is_tty and self._stdin_old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                                  self._stdin_old_settings)
            except Exception:
                pass

    # ------------------------------------------------------------ state m/c

    def _enter_stage(self, stage):
        if self.current_stage != stage:
            self.current_stage = stage
            self.stage_enter_time = time.monotonic()

    def _in_stage_for(self):
        return time.monotonic() - self.stage_enter_time

    def _begin_landing(self, reason):
        self.get_logger().warning(f"Landing: {reason}")
        self.landed_since = None
        # Flow degrades as we approach the ground, so stop chasing a latched
        # x/y point and go back to "just don't translate".
        if self.hold_xy:
            self.get_logger().info("Horizontal control -> zero-velocity hold for descent.")
        self.hold_xy = False
        self._enter_stage(self.LANDING)

    def timer_callback(self):
        # Heartbeat + setpoint go out on every tick, in every stage, so
        # Offboard never times out mid-flight.
        self.publish_offboard_control_mode()
        self.publish_position_setpoint()

        if self.kill_requested and self.current_stage not in (self.KILLING, self.DONE):
            self._enter_stage(self.KILLING)
        elif self.abort_requested and self.current_stage in (
                self.GROUND_WAIT, self.TAKEOFF, self.HOLD):
            self.abort_requested = False
            self._begin_landing("operator abort")

        handler = {
            self.PREPARATION: self._handle_preparation,
            self.OFFBOARD_REQUEST: self._handle_offboard_request,
            self.ARMING: self._handle_arming,
            self.GROUND_WAIT: self._handle_ground_wait,
            self.TAKEOFF: self._handle_takeoff,
            self.HOLD: self._handle_hold,
            self.LANDING: self._handle_landing,
            self.DISARMING: self._handle_disarming,
            self.KILLING: self._handle_killing,
            self.DONE: self._handle_done,
        }[self.current_stage]
        handler()

    def _handle_preparation(self):
        if not self.status_received:
            self.get_logger().info("Waiting for VehicleStatus from PX4...",
                                   throttle_duration_sec=2.0)
            return

        if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().error("Vehicle already armed at startup. Disarming.")
            self._enter_stage(self.DISARMING)
            return

        # No vertical estimate, no flight. This is the one hard gate.
        if not self.position_is_usable():
            self.get_logger().error(
                "Local position not valid (need xy_valid and z_valid). Not arming.",
                throttle_duration_sec=2.0)
            self.log_flight_state()
            self.setpoint_counter = 0
            return

        self.log_flight_state()

        self.setpoint_counter += 1
        if self.setpoint_counter >= self.SETPOINT_WARMUP:
            self.get_logger().info("Setpoint stream established, position estimate healthy.")
            self._enter_stage(self.OFFBOARD_REQUEST)

    def _handle_offboard_request(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("Offboard mode active.")
            self._enter_stage(self.ARMING)
            return

        if self.REQUEST_OFFBOARD_FROM_ROS:
            self.get_logger().info("Requesting Offboard mode...", throttle_duration_sec=1.0)
            # param1 = 1 -> custom mode enabled, param2 = 6 -> PX4 OFFBOARD
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        else:
            self.get_logger().info("Waiting for you to flip the Offboard switch on the TX...",
                                   throttle_duration_sec=2.0)

        if self._in_stage_for() > self.OFFBOARD_TIMEOUT:
            self.get_logger().error("Offboard mode not entered in time. Aborting.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)

    def _handle_arming(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self._capture_home()
            self.get_logger().warning(
                f"ARMED. Holding on the ground for {self.GROUND_WAIT_SECONDS:.0f} s.")
            self._enter_stage(self.GROUND_WAIT)
            return

        # Lost Offboard before we got the chance to arm.
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().error("Dropped out of Offboard. Aborting.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)
            return

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        if self._in_stage_for() > self.ARMING_TIMEOUT:
            self.get_logger().error("Arming rejected / timed out. Aborting.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)

    def _capture_home(self):
        # Only z and yaw are actually flown from this. x/y are recorded for
        # logging only -- the ground x/y estimate is not trustworthy enough
        # to be a setpoint (see the module docstring).
        lp = self.local_position
        self.home_x = lp.x
        self.home_y = lp.y
        self.home_z = lp.z
        self.home_yaw = lp.heading
        self.target_z = lp.z
        self.setpoint_z = lp.z
        self.get_logger().info(
            f"Arming point captured: x={self.home_x:.2f} y={self.home_y:.2f} "
            f"z={self.home_z:.2f} yaw={self.home_yaw:+.2f} rad")

    def _handle_ground_wait(self):
        if not self._still_flyable():
            return

        remaining = self.GROUND_WAIT_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self.get_logger().warning(
                f"Climbing to {self.TAKEOFF_ALTITUDE:.2f} m.")
            self.target_z = self.home_z - self.TAKEOFF_ALTITUDE
            self.in_band_since = None
            self._enter_stage(self.TAKEOFF)
            return

        self.target_z = self.home_z
        self.get_logger().info(f"Armed on the ground, {remaining:.1f} s to takeoff...",
                               throttle_duration_sec=1.0)
        self.log_flight_state()

    def _handle_takeoff(self):
        if not self._still_flyable():
            return

        self.log_flight_state()

        alt = self.relative_altitude()
        if alt is not None and abs(alt - self.TAKEOFF_ALTITUDE) <= self.ALTITUDE_TOLERANCE:
            if self.in_band_since is None:
                self.in_band_since = time.monotonic()
            elif time.monotonic() - self.in_band_since >= self.SETTLE_SECONDS:
                self.get_logger().warning(
                    f"Reached {alt:.2f} m. Holding for {self.HOLD_SECONDS:.0f} s.")
                self._enter_stage(self.HOLD)
            return

        self.in_band_since = None

        if self._in_stage_for() > self.TAKEOFF_TIMEOUT:
            self._begin_landing("takeoff did not settle in time")

    def _handle_hold(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_landing("hold complete")
            return

        self.get_logger().info(f"Holding, {remaining:.1f} s remaining...",
                               throttle_duration_sec=1.0)
        self.log_flight_state()

    def _try_latch_xy_hold(self):
        """Once airborne with good flow, anchor x/y to a fresh estimate.

        Zero-velocity hold drifts slowly with the flow's velocity bias. Once
        the estimate is being corrected by flow we can do better by holding an
        actual point -- but only a point sampled up here, never the one from
        the ground. If flow drops out later we fall back to velocity hold.
        """
        if not self.hold_xy:
            if self.flow_healthy_for() >= self.FLOW_SETTLE_SECONDS:
                self.hold_x = self.local_position.x
                self.hold_y = self.local_position.y
                self.hold_xy = True
                self.get_logger().info(
                    f"Flow healthy: latching x/y hold at "
                    f"({self.hold_x:.2f}, {self.hold_y:.2f}).")
        elif not self.flow_is_healthy():
            self.hold_xy = False
            self.get_logger().warning(
                "Flow unhealthy: reverting to zero-velocity hold.")

    def _handle_landing(self):
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Disarmed during descent. Done.")
            self._enter_stage(self.DONE)
            return

        # Walk the setpoint below the arming point so the vehicle keeps
        # pushing down into the ground instead of hovering just above it.
        self.target_z = self.home_z + self.LAND_OVERSHOOT
        self.log_flight_state()

        if self._touchdown_confirmed():
            self.get_logger().warning("Touchdown detected. Disarming.")
            self._enter_stage(self.DISARMING)
            return

        if self._in_stage_for() > self.LANDING_TIMEOUT:
            self.get_logger().error("Landing timed out. Disarming anyway.")
            self._enter_stage(self.DISARMING)

    def _touchdown_confirmed(self):
        """Land detector if we have one, otherwise altitude + descent stall."""
        if self.land_detector_seen:
            touched = self.landed
        else:
            lp = self.local_position
            alt = self.relative_altitude()
            touched = (alt is not None and alt < 0.10
                       and lp is not None and abs(lp.vz) < 0.10)

        if not touched:
            self.landed_since = None
            return False

        if self.landed_since is None:
            self.landed_since = time.monotonic()
        return time.monotonic() - self.landed_since >= self.LANDED_CONFIRM_SECONDS

    def _handle_disarming(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().info("Disarmed. Flight complete.")
            self._enter_stage(self.DONE)
            return

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

        if self._in_stage_for() > self.DISARM_TIMEOUT:
            self.get_logger().error("Normal disarm ignored. Escalating to force disarm.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)

    def _handle_killing(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().warning("Aborted; vehicle is disarmed.")
            self._enter_stage(self.DONE)
            return

        # param2 = 21196 is the PX4/MAVLink "force" magic number.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0, param2=21196.0)

    def _handle_done(self):
        self.get_logger().info("Idle. Ctrl-C to exit.", throttle_duration_sec=5.0)

    def _still_flyable(self):
        """Common bail-outs for every stage where the vehicle is under our control."""
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning("Vehicle disarmed by PX4. Stopping.")
            self._enter_stage(self.DONE)
            return False

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # PX4 (or the pilot) took the aircraft off us -- stop commanding it.
            self.get_logger().error("Offboard lost; PX4 has control now. Standing down.")
            self._enter_stage(self.DONE)
            return False

        if not self.position_is_usable():
            self._begin_landing("position estimate went invalid")
            return False

        return True

    # ------------------------------------------------------------ publishers

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # Both flags on: PX4 selects per-axis from which fields are NaN, so
        # we can fly z as a position and x/y as a velocity in one setpoint.
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self):
        """Ramp z as a position; hold x/y as either a velocity or a point.

        NaN in a TrajectorySetpoint field means "do not control this axis",
        which is what lets one message mix the two.
        """
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.home_z is None:
            # Not armed yet: warm the stream up holding the live altitude and
            # zero horizontal velocity, so nothing moves when Offboard engages.
            lp = self.local_position
            if lp is None:
                return
            msg.position = [nan, nan, lp.z]
            msg.velocity = [0.0, 0.0, nan]
            msg.yaw = lp.heading
            self.trajectory_setpoint_pub.publish(msg)
            return

        self._step_setpoint_ramp()

        if self.hold_xy:
            # Latched in flight on a flow-corrected estimate.
            msg.position = [self.hold_x, self.hold_y, self.setpoint_z]
            msg.velocity = [nan, nan, nan]
        else:
            # "Stay still." Memoryless: a jump in the position estimate cannot
            # be flown out, and an initial tilt off uneven ground just gets
            # corrected as soon as it produces velocity.
            msg.position = [nan, nan, self.setpoint_z]
            msg.velocity = [0.0, 0.0, nan]

        msg.yaw = self.home_yaw
        self.trajectory_setpoint_pub.publish(msg)

    def _step_setpoint_ramp(self):
        """Move the commanded z one timer tick towards target_z.

        Ramping instead of jumping straight to the target keeps the climb and
        especially the descent gentle -- the vehicle chases a setpoint that is
        never more than a fraction of a metre away from it.
        """
        dt = 0.05
        descending = self.target_z > self.setpoint_z
        speed = self.LAND_SPEED if descending else self.CLIMB_SPEED
        step = speed * dt

        delta = self.target_z - self.setpoint_z
        if abs(delta) <= step:
            self.setpoint_z = self.target_z
        else:
            self.setpoint_z += step if delta > 0 else -step

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
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        self._restore_stdin()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
