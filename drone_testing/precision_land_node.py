#!/usr/bin/env python3
"""Takeoff to 1.2 m, then descend onto the contrast pad under camera guidance.

Flight side of the pad-landing system. pad_detector_node does the looking; this
node does the flying, consuming /landing/ready, /landing/nudge and
/landing/height.

Sequence:
  WAIT_OFFBOARD -> ARMING -> TAKEOFF (1.2 m) -> SETTLE -> DESCEND -> TOUCHDOWN

DESCEND walks the position setpoint down at descent_rate (10 cm/s) for as long
as the detector says DESCEND. Lose the lock and it stops descending and holds
altitude - it does not keep going blind. Regain it and the descent resumes.

Below the detector's cutoff height (25 cm) the camera can no longer resolve the
pad, so the detector reports LAND and this node hands off to PX4's NAV_LAND for
the last few centimetres onto ~12 cm landing gear.

The pilot keeps authority throughout: leaving Offboard aborts immediately, and
the RC kill switch overrides everything.

Frames: PX4 local position is NED, so "up" is negative z.
"""

import csv
import math
import os
from datetime import datetime
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32, String


class State(Enum):
    WAIT_OFFBOARD = 'WAIT_OFFBOARD'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    SETTLE = 'SETTLE'
    DESCEND = 'DESCEND'
    SEARCH = 'SEARCH'
    BLIND_DESCEND = 'BLIND_DESCEND'
    TOUCHDOWN = 'TOUCHDOWN'
    DONE = 'DONE'
    ABORT = 'ABORT'


class PrecisionLandNode(Node):

    LOOP_HZ = 20.0

    CSV_COLUMNS = [
        't', 'state', 'armed',
        'ekf_alt',          # m above the arming point
        'pad_height',       # m, detector's fused height
        'setpoint_alt',     # m, commanded altitude above arming point
        'descent_cmd',      # m/s actually being applied
        'ready', 'status',  # detector verdict
        'nudge_x', 'nudge_y',
        'drift', 'drift_x', 'drift_y',
        'speed', 'vx', 'vy', 'vz',
        'heading', 'setpoint_yaw',
    ]

    def __init__(self):
        super().__init__('precision_land_node')

        self.declare_parameter('takeoff_altitude', 1.5)
        self.declare_parameter('settle_time', 1.5)        # s stable before descending
        self.declare_parameter('altitude_tolerance', 0.15)
        self.declare_parameter('descent_rate', 0.10)      # m/s - 10 cm/s
        self.declare_parameter('takeoff_timeout', 25.0)
        self.declare_parameter('arm_timeout', 10.0)
        self.declare_parameter('descend_timeout', 180.0)  # s before giving up and landing
        # How long the detector may go quiet before we stop descending. The
        # camera runs ~17 Hz, so this is several frames of grace, not a hair
        # trigger - but short enough that we never descend blind.
        self.declare_parameter('lock_timeout', 1.5)
        # Apply the detector's lateral nudges while descending.
        # If the pad cannot be seen for this long, stop waiting and come down
        # anyway. Hovering indefinitely over a pad we cannot see is the worse
        # failure: the battery runs out eventually and that landing is not
        # controlled at all.
        self.declare_parameter('lost_lock_timeout', 10.0)
        # --- lawnmower search ---
        # Run a T-pattern at the current altitude when the pad is out of sight,
        # before giving up and descending blind.
        self.declare_parameter('search_enabled', True)
        self.declare_parameter('search_step', 1.0)        # m per leg
        # How many rings to expand through before giving up. Coverage reaches
        # search_step * search_rings metres from the takeoff point.
        self.declare_parameter('search_rings', 2)
        self.declare_parameter('search_speed', 0.40)      # m/s while searching
        self.declare_parameter('search_settle', 0.8)      # s to pause and look at each stop
        self.declare_parameter('search_tolerance', 0.25)  # m, "arrived at waypoint"
        self.declare_parameter('blind_descent_rate', 0.10)  # m/s once blind
        self.declare_parameter('use_nudge', True)
        self.declare_parameter('max_nudge_rate', 0.10)    # m/s cap on lateral correction
        # Fall back to NAV_LAND at this height even if the detector never says LAND.
        self.declare_parameter('land_height', 0.25)
        # Touchdown detection independent of any height reference. On the first
        # flight the setpoint walked 0.5 m underground while the drone sat on the
        # floor, because pad_height (ekf_z, measured from the arming point) never
        # crossed land_height. The vehicle then slid 0.84 m on its gear. If
        # altitude stops responding while we are commanding descent, we are down.
        self.declare_parameter('stall_tolerance', 0.04)   # m of altitude change
        self.declare_parameter('stall_time', 1.5)         # s before calling it landed
        self.declare_parameter('log_csv', True)
        self.declare_parameter('log_dir', '/home/ark-jetson-orin/Downloads/cam/lend/logs')
        self.declare_parameter('report_period', 1.0)
        self.declare_parameter('vehicle_status_topic', '/fmu/out/vehicle_status_v1')
        self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1')

        self.takeoff_alt = float(self.get_parameter('takeoff_altitude').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.alt_tol = float(self.get_parameter('altitude_tolerance').value)
        self.descent_rate = float(self.get_parameter('descent_rate').value)
        self.takeoff_timeout = float(self.get_parameter('takeoff_timeout').value)
        self.arm_timeout = float(self.get_parameter('arm_timeout').value)
        self.descend_timeout = float(self.get_parameter('descend_timeout').value)
        self.lock_timeout = float(self.get_parameter('lock_timeout').value)
        self.lost_lock_timeout = float(self.get_parameter('lost_lock_timeout').value)
        self.search_enabled = bool(self.get_parameter('search_enabled').value)
        self.search_step = float(self.get_parameter('search_step').value)
        self.search_rings = int(self.get_parameter('search_rings').value)
        self.search_speed = float(self.get_parameter('search_speed').value)
        self.search_settle = float(self.get_parameter('search_settle').value)
        self.search_tolerance = float(self.get_parameter('search_tolerance').value)
        self.blind_descent_rate = float(self.get_parameter('blind_descent_rate').value)
        self.use_nudge = bool(self.get_parameter('use_nudge').value)
        self.max_nudge_rate = float(self.get_parameter('max_nudge_rate').value)
        self.land_height = float(self.get_parameter('land_height').value)
        self.stall_tolerance = float(self.get_parameter('stall_tolerance').value)
        self.stall_time = float(self.get_parameter('stall_time').value)
        self.log_csv = bool(self.get_parameter('log_csv').value)
        self.log_dir = os.path.expanduser(str(self.get_parameter('log_dir').value))
        self.report_period = float(self.get_parameter('report_period').value)
        status_topic = self.get_parameter('vehicle_status_topic').value
        position_topic = self.get_parameter('local_position_topic').value

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        self.create_subscription(
            VehicleLocalPosition, position_topic, self.on_local_position, px4_qos)
        self.create_subscription(VehicleStatus, status_topic, self.on_status, px4_qos)

        self.create_subscription(Bool, '/landing/ready', self.on_ready, 10)
        self.create_subscription(String, '/landing/status', self.on_status_msg, 10)
        self.create_subscription(Point, '/landing/nudge', self.on_nudge, 10)
        self.create_subscription(Float32, '/landing/height', self.on_pad_height, 10)

        self.status = None
        self.local_pos = None
        self.state = State.WAIT_OFFBOARD
        self.state_entered = self.now()
        self.last_command_sent = 0.0
        self.last_report = 0.0
        self.last_link_warn = 0.0

        self.origin = None
        self.origin_yaw = 0.0
        self.setpoint = None
        self.setpoint_yaw = 0.0
        self.settled_since = None

        # Detector state.
        self.pad_ready = False
        self.pad_status = 'no detector'
        self.pad_height = None
        self.nudge = (0.0, 0.0)
        self.last_detector_msg = 0.0
        self.descent_cmd = 0.0
        self.detector_warned = False
        # When the detector last said "go". Drives the failsafe: if this goes
        # stale for lost_lock_timeout we stop waiting for vision entirely.
        self.last_good_lock = self.now()
        self.stall_ref_alt = None
        self.stall_since = None

        # Search state: a list of body-frame offsets from the search origin.
        self.search_origin = None
        self.search_legs = []
        self.search_index = 0
        self.search_arrived = None
        self.searches_run = 0

        self.csv_file = self.csv_writer = self.csv_path = None
        self.log_opened = 0.0
        self.log_rows = 0

        self.get_logger().warn(
            f'precision_land_node up. Takeoff {self.takeoff_alt:.2f} m, then '
            f'camera-guided descent at {self.descent_rate * 100:.0f} cm/s. '
            f'Waiting for RC Offboard.')

        self.create_timer(1.0 / self.LOOP_HZ, self.tick)

    # ---------- helpers ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def px4_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def elapsed(self):
        return self.now() - self.state_entered

    def transition(self, new_state):
        self.get_logger().info(f'{self.state.value} -> {new_state.value}')
        self.state = new_state
        self.state_entered = self.now()
        self.last_command_sent = 0.0

    def is_offboard(self):
        return (self.status is not None
                and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def is_armed(self):
        return (self.status is not None
                and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def ekf_altitude(self):
        if self.local_pos is None or self.origin is None:
            return None
        return self.origin[2] - self.local_pos.z

    def detector_live(self):
        return (self.now() - self.last_detector_msg) < self.lock_timeout

    def pad_locked(self):
        """True only when the detector is BOTH fresh and saying go.

        pad_ready is a latched value from the last message received. If the
        camera stalls, that flag stays True while no new messages arrive, which
        made the last flight log ready=1 for 21.5 s with a dead detector. Always
        ask this, never pad_ready alone.
        """
        return self.detector_live() and self.pad_ready

    # ---------- subscriptions ----------

    def on_local_position(self, msg):
        self.local_pos = msg

    def on_status(self, msg):
        self.status = msg

    def on_ready(self, msg):
        self.pad_ready = bool(msg.data)
        self.last_detector_msg = self.now()
        if self.pad_ready:
            self.last_good_lock = self.now()

    def touchdown_detected(self, alt):
        """True when commanded descent stops producing altitude change.

        The only touchdown signal that does not depend on knowing true ground
        height: if we are pushing the setpoint down and the vehicle is not
        following, it is resting on something.
        """
        if alt is None or self.descent_cmd <= 0.0:
            self.stall_ref_alt = alt
            self.stall_since = None
            return False
        if self.stall_ref_alt is None:
            self.stall_ref_alt = alt
            self.stall_since = self.now()
            return False
        if abs(alt - self.stall_ref_alt) > self.stall_tolerance:
            self.stall_ref_alt = alt
            self.stall_since = self.now()
            return False
        return (self.stall_since is not None
                and self.now() - self.stall_since > self.stall_time)

    def lock_lost_for(self):
        """Seconds since the detector last cleared us to descend."""
        return self.now() - self.last_good_lock

    def on_status_msg(self, msg):
        self.pad_status = msg.data

    def on_nudge(self, msg):
        self.nudge = (float(msg.x), float(msg.y))

    def on_pad_height(self, msg):
        v = float(msg.data)
        self.pad_height = None if math.isnan(v) else v

    # ---------- PX4 output ----------

    def send_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self.px4_timestamp()
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self.px4_timestamp()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self):
        if self.setpoint is None:
            return
        msg = TrajectorySetpoint()
        msg.timestamp = self.px4_timestamp()
        msg.position = [float(v) for v in self.setpoint]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.yaw = float(self.setpoint_yaw)
        self.setpoint_pub.publish(msg)

    # ---------- CSV ----------

    def open_csv(self):
        if not self.log_csv or self.csv_writer is not None:
            return
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_path = os.path.join(self.log_dir, f'precland_{stamp}.csv')
            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.CSV_COLUMNS)
            self.log_opened = self.now()
            self.get_logger().info(f'logging to {self.csv_path}')
        except OSError as exc:
            self.get_logger().error(f'could not open log: {exc}. Continuing.')
            self.csv_file = self.csv_writer = None

    def log_row(self):
        if self.csv_writer is None or self.local_pos is None:
            return
        lp = self.local_pos
        ekf = self.ekf_altitude()
        dx = dy = drift = float('nan')
        if self.origin is not None:
            dx, dy = lp.x - self.origin[0], lp.y - self.origin[1]
            drift = math.hypot(dx, dy)
        speed = math.hypot(lp.vx, lp.vy) if lp.v_xy_valid else float('nan')
        sp_alt = (self.origin[2] - self.setpoint[2]
                  if (self.setpoint and self.origin) else None)

        def f(v, p=3):
            return '' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v:.{p}f}'

        self.csv_writer.writerow([
            f'{self.now() - self.log_opened:.3f}', self.state.value, int(self.is_armed()),
            f(ekf), f(self.pad_height), f(sp_alt), f(self.descent_cmd),
            int(self.pad_ready), self.pad_status,
            f(self.nudge[0]), f(self.nudge[1]),
            f(drift), f(dx), f(dy), f(speed), f(lp.vx), f(lp.vy), f(lp.vz),
            f(lp.heading, 4), f(self.setpoint_yaw, 4),
        ])
        self.log_rows += 1

    def close_csv(self):
        if self.csv_file is None:
            return
        try:
            self.csv_file.close()
            self.get_logger().info(
                f'log written: {self.csv_path} ({self.log_rows} rows)')
        finally:
            self.csv_file = self.csv_writer = None

    def report(self):
        if self.now() - self.last_report < self.report_period:
            return
        self.last_report = self.now()
        ekf = self.ekf_altitude()
        ekf_s = f'{ekf:.2f} m' if ekf is not None else 'n/a'
        pad_s = f'{self.pad_height:.2f} m' if self.pad_height is not None else 'n/a'
        self.get_logger().info(
            f'[{self.state.value}] alt={ekf_s} pad_h={pad_s} '
            f'v={self.descent_cmd * 100:+.0f}cm/s  detector: {self.pad_status}')

    # ---------- main loop ----------

    def tick(self):
        if self.state in (State.DONE, State.ABORT):
            return

        if self.status is None or self.local_pos is None:
            if self.now() - self.last_link_warn >= 5.0:
                self.last_link_warn = self.now()
                missing = []
                if self.status is None:
                    missing.append('VehicleStatus')
                if self.local_pos is None:
                    missing.append('VehicleLocalPosition')
                self.get_logger().warn(
                    f'no {" or ".join(missing)} - is the uXRCE-DDS agent running?')
            return

        if (self.state in (State.ARMING, State.TAKEOFF, State.SETTLE,
                           State.DESCEND, State.SEARCH, State.BLIND_DESCEND)
                and not self.is_offboard()):
            self.get_logger().warn('Offboard dropped - pilot has control. Aborting.')
            self.descent_cmd = 0.0
            self.transition(State.ABORT)
            return

        if self.local_pos is not None and self.origin is None:
            self.setpoint = (self.local_pos.x, self.local_pos.y, self.local_pos.z)
            self.setpoint_yaw = self.local_pos.heading

        self.publish_offboard_heartbeat()
        if self.state is not State.TOUCHDOWN:
            self.publish_setpoint()
        self.report()
        self.log_row()

        handler = {
            State.WAIT_OFFBOARD: self.do_wait_offboard,
            State.ARMING: self.do_arming,
            State.TAKEOFF: self.do_takeoff,
            State.SETTLE: self.do_settle,
            State.DESCEND: self.do_descend,
            State.SEARCH: self.do_search,
            State.BLIND_DESCEND: self.do_blind_descend,
            State.TOUCHDOWN: self.do_touchdown,
        }.get(self.state)
        if handler is not None:
            handler()

    def do_wait_offboard(self):
        if self.is_offboard():
            self.transition(State.ARMING)

    def do_arming(self):
        if self.is_armed():
            self.origin = (self.local_pos.x, self.local_pos.y, self.local_pos.z)
            self.origin_yaw = self.local_pos.heading
            self.setpoint_yaw = self.origin_yaw
            self.setpoint = (self.origin[0], self.origin[1],
                             self.origin[2] - self.takeoff_alt)
            self.open_csv()
            self.get_logger().info(f'Armed. Climbing to {self.takeoff_alt:.2f} m.')
            self.transition(State.TAKEOFF)
            return
        if self.elapsed() > self.arm_timeout:
            self.get_logger().error('Arm rejected within timeout. Aborting.')
            self.transition(State.ABORT)
            return
        if self.now() - self.last_command_sent >= 1.0:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.last_command_sent = self.now()

    def do_takeoff(self):
        alt = self.ekf_altitude()
        if alt is None:
            return
        self.setpoint_yaw=self.local_pos.heading
        if abs(alt - self.takeoff_alt) <= self.alt_tol:
            self.get_logger().info(f'Reached {alt:.2f} m. Settling.')
            self.transition(State.SETTLE)
            return
        if self.elapsed() > self.takeoff_timeout:
            self.get_logger().error(
                f'Did not reach {self.takeoff_alt:.2f} m (at {alt:.2f} m). Landing.')
            self.transition(State.TOUCHDOWN)

    def do_settle(self):
        """Hold still long enough for the hover to stabilise and the pad to lock."""
        alt = self.ekf_altitude()
        if alt is None:
            return
        if abs(alt - self.takeoff_alt) > self.alt_tol:
            self.settled_since = None
            return
        if self.settled_since is None:
            self.settled_since = self.now()

        if self.now() - self.settled_since < self.settle_time:
            return

        if not self.pad_locked():
            # Pad partly in view but below the coverage bar: steer toward it
            # rather than sitting still. The detector keeps publishing a nudge
            # whenever there is void to close, so this is how a half-framed pad
            # gets pulled fully into the circle before descending.
            self.apply_nudge()
            if not self.detector_warned and self.elapsed() > 5.0:
                self.detector_warned = True
                self.get_logger().error(
                    'No usable detection - is pad_detector_node running, and is '
                    'the pad under the camera? Holding.')
            # Never acquired a lock at all: the same failsafe applies, otherwise
            # we would sit at takeoff altitude forever.
            if not self.pad_locked() and self.lock_lost_for() > self.lost_lock_timeout:
                if not self.begin_search(
                        f'No detection for {self.lock_lost_for():.0f} s'):
                    self.get_logger().error('FAILSAFE: descending blind.')
                    self.transition(State.BLIND_DESCEND)
            return

        self.get_logger().info(
            f'Settled. Detector says "{self.pad_status}". Beginning descent.')
        self.transition(State.DESCEND)

    def do_descend(self):
        """Walk the setpoint down while the detector keeps saying yes."""
        alt = self.ekf_altitude()
        if alt is None:
            return

        if self.touchdown_detected(alt):
            self.get_logger().info(
                f'Altitude stopped responding at {alt:.2f} m while descending - '
                f'on the ground. NAV_LAND to disarm.')
            self.descent_cmd = 0.0
            self.transition(State.TOUCHDOWN)
            return

        # Height floor: hand off to PX4 for the last stretch. Prefer the pad
        # detector's own height, fall back to the EKF's.
        height = self.pad_height if self.pad_height is not None else alt
        if height <= self.land_height:
            self.get_logger().info(
                f'At {height * 100:.0f} cm - handing off to NAV_LAND.')
            self.descent_cmd = 0.0
            self.transition(State.TOUCHDOWN)
            return

        if self.elapsed() > self.descend_timeout:
            self.get_logger().warn(
                f'Descent timeout at {height:.2f} m. Landing anyway.')
            self.descent_cmd = 0.0
            self.transition(State.TOUCHDOWN)
            return

        # A currently-good lock always wins: never leave a pad we can see.
        # Keeps last_good_lock fresh so the failsafe below cannot fire while the
        # detector is actively saying DESCEND.
        if self.pad_locked():
            self.last_good_lock = self.now()

        # Failsafe: held here too long without a lock - come down blind rather
        # than hover until the battery decides for us.
        if not self.pad_locked() and self.lock_lost_for() > self.lost_lock_timeout:
            if self.begin_search(
                    f'No detection for {self.lock_lost_for():.0f} s at {height:.2f} m'):
                return
            self.get_logger().error('FAILSAFE: descending blind.')
            self.transition(State.BLIND_DESCEND)
            return

        # Descend only while the detector is both alive and saying go. Losing
        # the lock holds altitude rather than continuing blind.
        if not self.detector_live():
            self.descent_cmd = 0.0
            return
        if not self.pad_locked():
            self.descent_cmd = 0.0
            # Still apply lateral corrections - that is how we get the lock back.
            self.apply_nudge()
            return

        self.descent_cmd = self.descent_rate
        dt = 1.0 / self.LOOP_HZ
        self.setpoint = (self.setpoint[0], self.setpoint[1],
                         self.setpoint[2] + self.descent_rate * dt)
        self.apply_nudge()

    def apply_nudge(self):
        """Fold the detector's lateral correction into the setpoint."""
        if not self.use_nudge or self.setpoint is None:
            return
        nx, ny = self.nudge
        if nx == 0.0 and ny == 0.0:
            return
        # The detector emits a step in metres; spread it over time so the
        # setpoint moves smoothly instead of jumping.
        dt = 1.0 / self.LOOP_HZ
        step = self.max_nudge_rate * dt
        mag = math.hypot(nx, ny)
        sx, sy = nx / mag * step, ny / mag * step
        c, s = math.cos(self.setpoint_yaw), math.sin(self.setpoint_yaw)
        self.setpoint = (self.setpoint[0] + sx * c - sy * s,
                         self.setpoint[1] + sx * s + sy * c,
                         self.setpoint[2])

    def build_search_pattern(self):
        """Expanding box sweep, in the body frame latched at arming.

        The pad's distance is unknown in a real mission, so the pattern must
        grow until it finds something rather than stop at a tuned box. Each
        ring is a square lap at radius n * search_step, walked as a lawnmower:
        out along the forward arm, across, back, across, and out again to the
        next ring. search_step is the ring SPACING - it is deliberately not a
        target distance, so no number here encodes where the pad happens to be.

        Spacing is what guarantees coverage: consecutive rings are one step
        apart, so any pad wider than search_step must fall inside some leg's
        camera footprint. Detection runs continuously along every leg (see
        do_search), not only at the corners, so the pad is caught in transit.

        Yaw never changes: every leg is a pure translation on the heading
        latched at arming, keeping the camera square to the ground throughout.

        Each entry is (dx_forward, dy_right) in metres, absolute from the
        search origin - not deltas.
        """
        step = self.search_step
        legs = []
        for ring in range(1, self.search_rings + 1):
            r = ring * step
            # Lawnmower lap of this ring: forward arm, sweep right, sweep back
            # along the rear arm, and return to the centre line ready to expand.
            legs.extend([
                (r, 0.0),      # out along the forward arm
                (r, -r),       # forward-left corner
                (-r, -r),      # sweep down the left side
                (-r, r),       # along the rear to the right
                (r, r),        # up the right side
                (r, 0.0),      # close the lap on the forward arm
                (0.0, 0.0),    # back through the origin, then expand
            ])
        return legs

    def begin_search(self, why):
        """Start a lawnmower sweep at the current altitude."""
        if not self.search_enabled or self.local_pos is None:
            return False
        self.search_origin = (self.setpoint[0], self.setpoint[1], self.setpoint[2])
        self.search_legs = self.build_search_pattern()
        self.search_index = 0
        self.search_arrived = None
        self.searches_run += 1
        self.descent_cmd = 0.0
        self.get_logger().warn(
            f'{why} - starting lawnmower search #{self.searches_run} at '
            f'{self.ekf_altitude():.2f} m ({len(self.search_legs)} waypoints).')
        self.transition(State.SEARCH)
        return True

    def do_search(self):
        """Fly the T-pattern, watching for the pad at every point.

        Altitude is held for the whole sweep - searching is a horizontal
        activity. Finding the pad at any moment abandons the pattern and goes
        straight back to descending.
        """
        # Found it - stop searching immediately, wherever we are in the pattern.
        # Checked before any state guard: last flight sat in SEARCH for 21.5 s
        # with ready=1 because a guard above this returned first.
        if self.pad_locked():
            self.get_logger().info(
                f'Pad reacquired during search ("{self.pad_status}"). Resuming descent.')
            self.last_good_lock = self.now()
            self.transition(State.DESCEND)
            return

        if self.local_pos is None or self.search_origin is None:
            return

        if self.search_index >= len(self.search_legs):
            # Whole pattern flown with nothing found.
            self.get_logger().error(
                'Lawnmower search found no pad. FAILSAFE: descending blind.')
            self.transition(State.BLIND_DESCEND)
            return

        dx, dy = self.search_legs[self.search_index]
        c, s_ = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        target = (self.search_origin[0] + dx * c - dy * s_,
                  self.search_origin[1] + dx * s_ + dy * c,
                  self.search_origin[2])

        # Walk the setpoint toward the waypoint at search_speed rather than
        # jumping it, so the vehicle tracks smoothly and the camera stays steady.
        step = self.search_speed / self.LOOP_HZ
        ex, ey = target[0] - self.setpoint[0], target[1] - self.setpoint[1]
        dist = math.hypot(ex, ey)
        if dist > step:
            self.setpoint = (self.setpoint[0] + ex / dist * step,
                             self.setpoint[1] + ey / dist * step,
                             target[2])
        else:
            self.setpoint = target

        # Arrived? Pause there so the detector gets a few clean frames.
        actual = math.hypot(self.local_pos.x - target[0], self.local_pos.y - target[1])
        if actual <= self.search_tolerance:
            if self.search_arrived is None:
                self.search_arrived = self.now()
            elif self.now() - self.search_arrived >= self.search_settle:
                self.search_index += 1
                self.search_arrived = None
        else:
            self.search_arrived = None

    def do_blind_descend(self):
        """Come straight down with no vision input.

        Deliberately ignores /landing/ready: we got here because that signal is
        unavailable or untrustworthy, so re-consulting it would just stall again.
        Height still gates the handoff to NAV_LAND, and the pilot's RC switch
        still aborts - this is a controlled descent, not a free fall.
        """
        alt = self.ekf_altitude()
        if alt is None:
            return

        if self.touchdown_detected(alt):
            self.get_logger().info(
                f'Altitude stopped responding at {alt:.2f} m while descending - '
                f'on the ground. NAV_LAND to disarm.')
            self.descent_cmd = 0.0
            self.transition(State.TOUCHDOWN)
            return

        height = self.pad_height if self.pad_height is not None else alt
        if height <= self.land_height:
            self.get_logger().info(
                f'Blind descent reached {height * 100:.0f} cm - NAV_LAND.')
            self.descent_cmd = 0.0
            self.transition(State.TOUCHDOWN)
            return

        if self.elapsed() > self.descend_timeout:
            self.get_logger().warn('Blind descent timeout. NAV_LAND.')
            self.descent_cmd = 0.0
            self.transition(State.TOUCHDOWN)
            return

        self.descent_cmd = self.blind_descent_rate
        dt = 1.0 / self.LOOP_HZ
        self.setpoint = (self.setpoint[0], self.setpoint[1],
                         self.setpoint[2] + self.blind_descent_rate * dt)

    def do_touchdown(self):
        if self.now() - self.last_command_sent >= 1.0 and self.is_armed():
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.last_command_sent = self.now()
        if not self.is_armed():
            self.get_logger().info('Landed and disarmed. Precision landing complete.')
            self.transition(State.DONE)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandNode()
    try:
        while rclpy.ok() and node.state not in (State.DONE, State.ABORT):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
